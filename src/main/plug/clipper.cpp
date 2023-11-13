/*
 * Copyright (C) 2023 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2023 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-clipper
 * Created on: 11 ноя 2023 г.
 *
 * lsp-plugins-clipper is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-clipper is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-clipper. If not, see <https://www.gnu.org/licenses/>.
 */

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/bits.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/dsp-units/misc/envelope.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>

#include <private/plugins/clipper.h>

/* The size of temporary buffer for audio processing */
#define BUFFER_SIZE         0x400U

namespace lsp
{
    namespace plugins
    {
        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::clipper_mono,
            &meta::clipper_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new clipper(meta);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //---------------------------------------------------------------------
        // Implementation
        clipper::clipper(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels       = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Initialize other parameters
            vChannels       = NULL;

            for (size_t i=0; i<meta::clipper::BANDS_MAX-1; ++i)
            {
                split_t *sp     = &vSplits[i];

                sp->fFreq       = 0.0f;
                sp->pFreq       = NULL;
            }

            enXOverMode     = XOVER_IIR;

            fInGain         = GAIN_AMP_0_DB;
            fOutGain        = GAIN_AMP_0_DB;
            fThresh         = GAIN_AMP_0_DB;

            pBypass         = NULL;
            pGainIn         = NULL;
            pGainOut        = NULL;

            pBypass         = NULL;
            pGainIn         = NULL;
            pGainOut        = NULL;
            pThresh         = NULL;
            pBoosting       = NULL;
            pXOverMode      = NULL;
            pXOverSlope     = NULL;
            pFftReactivity  = NULL;
            pFftShift       = NULL;
            pZoom           = NULL;
            pHpfSlope       = NULL;
            pHpfFreq        = NULL;
            pLpfSlope       = NULL;
            pLpfFreq        = NULL;
            pExtraBandOn    = NULL;
            pFilterCurves   = NULL;
            pStereoLink     = NULL;

            pData           = NULL;
        }

        clipper::~clipper()
        {
            do_destroy();
        }

        void clipper::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Estimate the number of bytes to allocate
            size_t szof_channels    = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            size_t szof_buffer      = align_size(sizeof(float) * BUFFER_SIZE, OPTIMAL_ALIGN);
            size_t alloc            =
                szof_channels +
                nChannels * (
                    szof_buffer +       // vData
                    szof_buffer +       // vInAnalyze
                    meta::clipper::BANDS_MAX * (
                        szof_buffer +       // vData
                        szof_buffer         // vInAnalyze
                    )
                );

            // Initialize analyzer
            size_t an_id            = 0;
            if (!sAnalyzer.init(nChannels * 2, meta::clipper::FFT_RANK,
                MAX_SAMPLE_RATE, meta::clipper::REFRESH_RATE))
                return;
            sAnalyzer.set_rank(meta::clipper::FFT_RANK);
            sAnalyzer.set_activity(false);
            sAnalyzer.set_envelope(dspu::envelope::WHITE_NOISE);
            sAnalyzer.set_window(meta::clipper::FFT_WINDOW);
            sAnalyzer.set_rate(meta::clipper::REFRESH_RATE);

            sCounter.set_frequency(meta::clipper::REFRESH_RATE, true);

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr<channel_t>(ptr, szof_channels);

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sEqualizer.construct();
                c->sIIRXOver.construct();
                c->sFFTXOver.construct();

                // Initialize equalizer
                if (!c->sEqualizer.init(2, 0))
                    return;
                if (!c->sIIRXOver.init(meta::clipper::BANDS_MAX, BUFFER_SIZE))
                    return;

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];

                    // Initialize DSP units
                    b->sSidechain.construct();
                    b->sScDelay.construct();
                    b->sDelay.construct();

                    // Bind handler to crossover
                    c->sIIRXOver.set_handler(j, process_band, this, c);

                    // Initialize fields
                    b->fInLevel             = GAIN_AMP_M_INF_DB;
                    b->fOutLevel            = GAIN_AMP_M_INF_DB;

                    b->vData                = advance_ptr<float>(ptr, szof_buffer);
                    b->vSc                  = advance_ptr<float>(ptr, szof_buffer);
                }

                // Initialize fields
                c->nAnInChannel         = an_id++;
                c->nAnOutChannel        = an_id++;
                c->fInGain              = GAIN_AMP_0_DB;
                c->fOutGain             = GAIN_AMP_0_DB;

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vData                = advance_ptr<float>(ptr, szof_buffer);
                c->vInAnalyze           = advance_ptr<float>(ptr, szof_buffer);

                c->pIn                  = NULL;
                c->pOut                 = NULL;
            }

            // Bind ports
            lsp_trace("Binding input ports");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pIn    = trace_port(ports[port_id++]);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pOut   = trace_port(ports[port_id++]);

            // Bind bypass
            lsp_trace("Binding common ports");
            pBypass             = trace_port(ports[port_id++]);
            pGainIn             = trace_port(ports[port_id++]);
            pGainOut            = trace_port(ports[port_id++]);
            pThresh             = trace_port(ports[port_id++]);
            pBoosting           = trace_port(ports[port_id++]);
            pXOverMode          = trace_port(ports[port_id++]);
            pXOverSlope         = trace_port(ports[port_id++]);
            pFftReactivity      = trace_port(ports[port_id++]);
            pFftShift           = trace_port(ports[port_id++]);
            pZoom               = trace_port(ports[port_id++]);
            pHpfSlope           = trace_port(ports[port_id++]);
            pHpfFreq            = trace_port(ports[port_id++]);
            for (size_t i=0; i<meta::clipper::BANDS_MAX-1; ++i)
                vSplits[i].pFreq    = trace_port(ports[port_id++]);
            pLpfSlope           = trace_port(ports[port_id++]);
            pLpfFreq            = trace_port(ports[port_id++]);
            pExtraBandOn        = trace_port(ports[port_id++]);
            pFilterCurves       = trace_port(ports[port_id++]);

            if (nChannels > 1)
            {
                pStereoLink         = trace_port(ports[port_id++]);
            }

            // Bind ports for audio processing channels
            for (size_t i=0; i<nChannels; ++i)
            {
//                channel_t *c            = &vChannels[i];
            }

            // Bind output meters
            for (size_t i=0; i<nChannels; ++i)
            {
//                channel_t *c            = &vChannels[i];
            }
        }

        void clipper::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void clipper::do_destroy()
        {
            // Destroy channels
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    c->sBypass.destroy();
                    c->sDryDelay.destroy();
                    c->sEqualizer.destroy();
                    c->sIIRXOver.destroy();
                    c->sFFTXOver.destroy();

                    for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                    {
                        band_t *b       = &c->vBands[j];

                        b->sSidechain.destroy();
                        b->sScDelay.destroy();
                        b->sDelay.destroy();
                    }
                }
                vChannels   = NULL;
            }

            // Free previously allocated data block
            free_aligned(pData);
        }

        size_t clipper::select_fft_rank(size_t sample_rate)
        {
            const size_t k = (sample_rate + meta::clipper::FFT_XOVER_FREQ_MIN/2) / meta::clipper::FFT_XOVER_FREQ_MIN;
            const size_t n = int_log2(k);
            return meta::clipper::FFT_XOVER_RANK_MIN + n;
        }

        void clipper::update_sample_rate(long sr)
        {
            const size_t fft_rank       = select_fft_rank(sr);
            const size_t max_delay_fft  = (1 << fft_rank);

            sCounter.set_sample_rate(sr, true);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.init(sr);
                c->sDryDelay.init(max_delay_fft);
                c->sEqualizer.set_sample_rate(sr);
                c->sIIRXOver.set_sample_rate(sr);

                if (fft_rank != c->sFFTXOver.rank())
                {
                    c->sFFTXOver.init(fft_rank, meta::clipper::BANDS_MAX);
                    for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                        c->sFFTXOver.set_handler(j, process_band, this, c);
                    c->sFFTXOver.set_rank(fft_rank);
                    c->sFFTXOver.set_phase(float(i) / float(nChannels));
                }
                c->sFFTXOver.set_sample_rate(sr);
            }

            // Commit sample rate to analyzer
            sAnalyzer.set_sample_rate(sr);
        }

        inline size_t clipper::filter_slope(size_t slope)
        {
            return slope;
        }

        inline float clipper::fft_filter_slope(size_t slope)
        {
            return -24.0f * slope;
        }

        void clipper::update_settings()
        {
            bool bypass             = pBypass->value() >= 0.5f;
            fThresh                 = dspu::db_to_gain(-pThresh->value());
            float out_gain          = pGainOut->value();

            fInGain                 = pGainIn->value();
            fOutGain                = (pBoosting->value() >= 0.5f) ? out_gain : out_gain * fThresh;

            enXOverMode             = (pXOverMode->value() >= 1) ? XOVER_FFT : XOVER_IIR;

            // Configure split frequencies
            const size_t num_splits = (pExtraBandOn->value() >= 0.5f) ? meta::clipper::BANDS_MAX-1 : meta::clipper::BANDS_MAX-2;
            for (size_t i=0; i<meta::clipper::BANDS_MAX-1; ++i)
            {
                split_t *sp             = &vSplits[i];
                sp->fFreq               = sp->pFreq->value();
            }

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.set_bypass(bypass);

                if (enXOverMode == XOVER_IIR)
                {
                    dspu::Crossover *xc     = &c->sIIRXOver;
                    size_t iir_slope        = filter_slope(pXOverSlope->value());

                    // Configure split points for crossover
                    for (size_t i=0; i<meta::clipper::BANDS_MAX-1; ++i)
                    {
                        split_t *sp         = &vSplits[i];

                        xc->set_frequency(i, sp->fFreq);
                        xc->set_mode(i, dspu::CROSS_MODE_BT);
                        xc->set_slope(i, (i < num_splits) ? iir_slope : 0);
                    }
                }
                else // (enXOverMode == XOVER_FFT)
                {
                    dspu::FFTCrossover *xf  = &c->sFFTXOver;
                    const float fft_slope   = fft_filter_slope(pXOverSlope->value());
                    const float hpf_slope   = fft_filter_slope(pHpfSlope->value());
                    const float lpf_slope   = fft_filter_slope(pLpfSlope->value());

                    // Configure split points for crossover
                    for (size_t i=0; i < meta::clipper::BANDS_MAX; ++i)
                    {
//                        band_t *b       = &c->vBands[i];

                        if (i > 0)
                        {
                            xf->enable_hpf(i, true);
                            xf->set_hpf_frequency(i, vSplits[i-1].fFreq);
                            xf->set_hpf_slope(i, fft_slope);
                        }
                        else
                        {
                            xf->enable_hpf(i, hpf_slope > 0.0f);
                            xf->set_hpf_frequency(i, pHpfFreq->value());
                            xf->set_hpf_slope(i, hpf_slope);
                        }

                        if (i < num_splits)
                        {
                            xf->enable_lpf(i, true);
                            xf->set_lpf_frequency(i, vSplits[i].fFreq);
                            xf->set_lpf_slope(i, fft_slope);
                        }
                        else
                        {
                            xf->enable_lpf(i, lpf_slope > 0.0f);
                            xf->set_lpf_frequency(i, pLpfFreq->value());
                            xf->set_lpf_slope(i, lpf_slope);
                        }

                        xf->enable_band(i, i < num_splits);
                    }
                }
            }
        }

        void clipper::process_band(void *object, void *subject, size_t band, const float *data, size_t sample, size_t count)
        {
            channel_t *c            = static_cast<channel_t *>(subject);
            band_t *b               = &c->vBands[band];

            // Apply delay compensation and store to band's data buffer.
            b->sDelay.process(&b->vData[sample], data, count);
            b->sScDelay.process(&b->vSc[sample], data, count);

            // Measure the input level
//            b->fInLevel             = lsp_max(dsp::abs_max(&b->vInData[sample], count), b->fInLevel);
        }

        void clipper::bind_input_buffers()
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                c->vIn              = c->pIn->buffer<float>();
                c->vOut             = c->pOut->buffer<float>();

                c->fInGain          = GAIN_AMP_M_INF_DB;
                c->fOutGain         = GAIN_AMP_M_INF_DB;
            }
        }

        void clipper::advance_buffers(size_t samples)
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                c->vIn             += samples;
                c->vOut            += samples;
            }
        }

        void clipper::perform_analysis(size_t samples)
        {
            // Prepare processing
            const float *bufs[4] = { NULL, NULL, NULL, NULL };
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                bufs[c->nAnInChannel]   = c->vInAnalyze;
                bufs[c->nAnOutChannel]  = c->vData;

//                c->pOutMeter->set_value(dsp::abs_max(c->vData, samples));
//                c->pInMeter->set_value(dsp::abs_max(c->vInBuf, samples) * fInGain);
            }

            // Perform FFT analysis
            sAnalyzer.process(bufs, samples);
        }

        void clipper::process(size_t samples)
        {
            bind_input_buffers();

            for (size_t offset = 0; offset < samples; )
            {
                size_t to_do    = lsp_min(samples - offset, BUFFER_SIZE);

                perform_analysis(to_do);

                advance_buffers(to_do);
                offset         += to_do;
            }
        }

        bool clipper::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // TODO
            return false;
        }

        void clipper::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            // TODO
            // It is very useful to dump plugin state for debug purposes
            v->write("nChannels", nChannels);
            v->begin_array("vChannels", vChannels, nChannels);
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                v->begin_object(c, sizeof(channel_t));
                {
                    v->write_object("sBypass", &c->sBypass);

                    v->write("vIn", c->vIn);
                    v->write("vOut", c->vOut);

                    v->write("pIn", c->pIn);
                    v->write("pOut", c->pOut);
                }
                v->end_object();
            }
            v->end_array();

            v->write("pBypass", pBypass);
            v->write("pGainIn", pGainIn);
            v->write("pGainOut", pGainOut);

            v->write("pData", pData);
        }

    } /* namespace plugins */
} /* namespace lsp */


