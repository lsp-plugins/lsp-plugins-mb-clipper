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

        dspu::sigmoid::function_t clipper::vSigmoidFunctions[] =
        {
            dspu::sigmoid::hard_clip,
            dspu::sigmoid::quadratic,
            dspu::sigmoid::sine,
            dspu::sigmoid::logistic,
            dspu::sigmoid::arctangent,
            dspu::sigmoid::hyperbolic_tangent,
            dspu::sigmoid::guidermannian,
            dspu::sigmoid::error,
            dspu::sigmoid::smoothstep,
            dspu::sigmoid::smootherstep,
            dspu::sigmoid::circle
        };

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
                sp->fOdpLink    = 0.0f;
                sp->pFreq       = NULL;
                sp->pOdpLink    = NULL;
            }

            for (size_t i=0; i<meta::clipper::BANDS_MAX; ++i)
            {
                processor_t *p          = &vProc[i];

                // Initialize fields
                p->sComp.x0             = 0.0f;
                p->sComp.x1             = 0.0f;
                p->sComp.x2             = 0.0f;
                p->sComp.t              = 0.0f;
                p->sComp.a              = 0.0f;
                p->sComp.b              = 0.0f;
                p->sComp.c              = 0.0f;

                p->sOdp.fThreshold      = 0.0f;
                p->sOdp.fKnee           = 0.0f;

                p->sOdp.pThreshold      = NULL;
                p->sOdp.pKnee           = NULL;
                p->sOdp.pResonance      = NULL;
                p->sOdp.pCurveMesh      = NULL;

                p->sClip.pFunc          = NULL;
                p->sClip.fThreshold     = 0.0f;
                p->sClip.fPumping       = 1.0f;
                p->sClip.fScaling       = 0.0f;
                p->sClip.fKnee          = 0.0f;

                p->sClip.pOn            = NULL;
                p->sClip.pFunction      = NULL;
                p->sClip.pThreshold     = NULL;
                p->sClip.pPumping       = NULL;
                p->sClip.pCurveMesh     = NULL;

                p->nFlags               = PF_DIRTY_BAND | PF_SYNC_ALL;
                p->fPreamp              = 0.0f;
                p->fStereoLink          = 0.0f;
                p->fMakeup              = 0.0f;

                p->vTr                  = NULL;

                p->pSolo                = NULL;
                p->pMute                = NULL;
                p->pPreamp              = NULL;
                p->pStereoLink          = NULL;
                p->pMakeup              = NULL;
                p->pFreqChart           = NULL;
            }

            sComp.x0                = 0.0f;
            sComp.x1                = 0.0f;
            sComp.x2                = 0.0f;
            sComp.t                 = 0.0f;
            sComp.a                 = 0.0f;
            sComp.b                 = 0.0f;
            sComp.c                 = 0.0f;

            sOdp.fThreshold         = 0.0f;
            sOdp.fKnee              = 0.0f;

            sOdp.pThreshold         = NULL;
            sOdp.pKnee              = NULL;
            sOdp.pResonance         = NULL;
            sOdp.pCurveMesh         = NULL;

            sClip.pFunc             = NULL;
            sClip.fThreshold        = 0.0f;
            sClip.fPumping          = 1.0f;
            sClip.fScaling          = 0.0f;
            sClip.fKnee             = 0.0f;

            sClip.pOn               = NULL;
            sClip.pFunction         = NULL;
            sClip.pThreshold        = NULL;
            sClip.pPumping          = NULL;
            sClip.pCurveMesh        = NULL;

            enXOverMode     = XOVER_IIR;

            fInGain         = GAIN_AMP_0_DB;
            fOutGain        = GAIN_AMP_0_DB;
            fThresh         = GAIN_AMP_0_DB;
            fStereoLink     = 0.0f;
            nFlags          = GF_SYNC_ALL;

            vBuffer         = NULL;
            vFreqs          = NULL;
            vIndexes        = NULL;
            vTrEq           = NULL;
            vOdp            = NULL;
            vLinSigmoid     = NULL;
            vLogSigmoid     = NULL;
            vTime           = NULL;

            pBypass         = NULL;
            pGainIn         = NULL;
            pGainOut        = NULL;
            pThresh         = NULL;
            pBoosting       = NULL;
            pStereoLink     = NULL;
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
            pOutClipperOn   = NULL;
            pFilterCurves   = NULL;

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
            size_t szof_fft_buffer  = align_size(sizeof(float) * meta::clipper::FFT_MESH_POINTS, OPTIMAL_ALIGN);
            size_t szof_idx_buffer  = align_size(sizeof(uint32_t) * meta::clipper::FFT_MESH_POINTS, OPTIMAL_ALIGN);
            size_t szof_curve_buffer= align_size(sizeof(float) * meta::clipper::CURVE_MESH_POINTS, OPTIMAL_ALIGN);
            size_t szof_time_buffer = align_size(sizeof(float) * meta::clipper::TIME_MESH_POINTS, OPTIMAL_ALIGN);
            size_t to_alloc         =
                szof_channels +
                szof_buffer +           // vBuffer
                szof_fft_buffer +       // vFreqs
                szof_idx_buffer +       // vIndexes
                szof_fft_buffer +       // vTrEq
                szof_curve_buffer +     // vOdp
                szof_curve_buffer +     // vLinSigmoid
                szof_curve_buffer +     // vLogSigmoid
                szof_time_buffer +      // vTime
                meta::clipper::BANDS_MAX * (
                    szof_fft_buffer     // vTr
                ) +
                nChannels * (
                    szof_buffer +       // vData
                    szof_buffer +       // vSc
                    szof_buffer +       // vInAnalyze
                    meta::clipper::BANDS_MAX * (
                        szof_buffer +       // vInData
                        szof_buffer         // vData
                    )
                );

            // Initialize analyzer
            size_t an_id            = 0;
            sCounter.set_frequency(meta::clipper::REFRESH_RATE, true);

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, to_alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;
            lsp_guard_assert( const uint8_t *tail = &ptr[to_alloc]; );

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);
            vBuffer                 = advance_ptr_bytes<float>(ptr, szof_buffer);
            vFreqs                  = advance_ptr_bytes<float>(ptr, szof_fft_buffer);
            vIndexes                = advance_ptr_bytes<uint32_t>(ptr, szof_idx_buffer);
            vTrEq                   = advance_ptr_bytes<float>(ptr, szof_fft_buffer);
            vOdp                    = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vLinSigmoid             = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vLogSigmoid             = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vTime                   = advance_ptr_bytes<float>(ptr, szof_time_buffer);

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sDryDelay.construct();
                c->sScDelay.construct();
                c->sSc.construct();
                c->sEqualizer.construct();
                c->sIIRXOver.construct();
                c->sFFTXOver.construct();

                // Initialize equalizer
                if (!c->sEqualizer.init(2, 0))
                    return;
                c->sEqualizer.set_mode(dspu::EQM_IIR);

                if (!c->sIIRXOver.init(meta::clipper::BANDS_MAX, BUFFER_SIZE))
                    return;

                c->sInGraph.construct();
                c->sOutGraph.construct();

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];

                    // Initialize DSP units
                    b->sSc.construct();
                    b->sScDelay.construct();
                    b->sInDelay.construct();
                    b->sPreDelay.construct();
                    b->sPostDelay.construct();
                    b->sInGraph.construct();
                    b->sOutGraph.construct();

                    // Bind handler to crossover
                    c->sIIRXOver.set_handler(j, process_band, this, c);

                    b->fIn                  = GAIN_AMP_M_INF_DB;
                    b->fOut                 = GAIN_AMP_M_INF_DB;
                    b->fRed                 = GAIN_AMP_M_INF_DB;

                    b->fOdpIn               = GAIN_AMP_M_INF_DB;
                    b->fOdpOut              = GAIN_AMP_M_INF_DB;
                    b->fOdpRed              = GAIN_AMP_M_INF_DB;

                    b->fClipIn              = GAIN_AMP_M_INF_DB;
                    b->fClipOut             = GAIN_AMP_M_INF_DB;
                    b->fClipRed             = GAIN_AMP_M_INF_DB;

                    b->vInData              = advance_ptr_bytes<float>(ptr, szof_buffer);
                    b->vData                = advance_ptr_bytes<float>(ptr, szof_buffer);

                    b->pIn                  = NULL;
                    b->pOut                 = NULL;
                    b->pRed                 = NULL;

                    b->pOdpIn               = NULL;
                    b->pOdpOut              = NULL;
                    b->pOdpRed              = NULL;

                    b->pClipIn              = NULL;
                    b->pClipOut             = NULL;
                    b->pClipRed             = NULL;

                    b->pTimeMesh            = NULL;
                }

                // Initialize fields
                c->nAnInChannel         = an_id++;
                c->nAnOutChannel        = an_id++;
                c->nFlags               = 0;

                c->fIn                  = GAIN_AMP_M_INF_DB;
                c->fOut                 = GAIN_AMP_M_INF_DB;
                c->fRed                 = GAIN_AMP_M_INF_DB;

                c->fOdpIn               = GAIN_AMP_M_INF_DB;
                c->fOdpOut              = GAIN_AMP_M_INF_DB;
                c->fOdpRed              = GAIN_AMP_M_INF_DB;

                c->fClipIn              = GAIN_AMP_M_INF_DB;
                c->fClipOut             = GAIN_AMP_M_INF_DB;
                c->fClipRed             = GAIN_AMP_M_INF_DB;

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vData                = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vSc                  = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vInAnalyze           = advance_ptr_bytes<float>(ptr, szof_buffer);

                // Initialize ports
                c->pDataIn              = NULL;
                c->pDataOut             = NULL;
                c->pFftInSwitch         = NULL;
                c->pFftOutSwitch        = NULL;
                c->pFftInMesh           = NULL;
                c->pFftOutMesh          = NULL;

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pRed                 = NULL;

                c->pOdpIn               = NULL;
                c->pOdpOut              = NULL;
                c->pOdpRed              = NULL;

                c->pClipIn              = NULL;
                c->pClipOut             = NULL;
                c->pClipRed             = NULL;

                c->pTimeMesh            = NULL;
            }

            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                processor_t *p          = &vProc[j];
                p->vTr                  = advance_ptr_bytes<float>(ptr, szof_fft_buffer);
            }

            lsp_assert( ptr <= tail );

            // Bind ports
            lsp_trace("Binding input ports");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pDataIn    = trace_port(ports[port_id++]);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pDataOut   = trace_port(ports[port_id++]);

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
            {
                split_t *sp         = &vSplits[i];
                sp->pFreq           = trace_port(ports[port_id++]);
                sp->pOdpLink        = trace_port(ports[port_id++]);
            }
            pLpfSlope           = trace_port(ports[port_id++]);
            pLpfFreq            = trace_port(ports[port_id++]);
            pExtraBandOn        = trace_port(ports[port_id++]);
            pOutClipperOn       = trace_port(ports[port_id++]);
            trace_port(ports[port_id++]); // Skip band selector
            pFilterCurves       = trace_port(ports[port_id++]);
            trace_port(ports[port_id++]); // Skip clipper linear/logarithmic graph view

            // Bind processor ports
            lsp_trace("Binding processor ports");
            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                processor_t *p          = &vProc[j];

                p->pStereoLink          = (nChannels > 1) ? trace_port(ports[port_id++]) : NULL;
                p->pSolo                = trace_port(ports[port_id++]);
                p->pMute                = trace_port(ports[port_id++]);
                p->pPreamp              = trace_port(ports[port_id++]);
                p->pMakeup              = trace_port(ports[port_id++]);
                p->sOdp.pOn             = trace_port(ports[port_id++]);
                p->sOdp.pThreshold      = trace_port(ports[port_id++]);
                p->sOdp.pKnee           = trace_port(ports[port_id++]);
                p->sOdp.pResonance      = trace_port(ports[port_id++]);
                p->sOdp.pCurveMesh      = trace_port(ports[port_id++]);
                p->sClip.pOn            = trace_port(ports[port_id++]);
                p->sClip.pFunction      = trace_port(ports[port_id++]);
                p->sClip.pThreshold     = trace_port(ports[port_id++]);
                p->sClip.pPumping       = trace_port(ports[port_id++]);
                p->sClip.pCurveMesh     = trace_port(ports[port_id++]);
                p->pFreqChart           = trace_port(ports[port_id++]);
            }

            // Bind output clipper ports
            lsp_trace("Binding output clipper ports");
            pStereoLink             = (nChannels > 1) ? trace_port(ports[port_id++]) : NULL;
            sOdp.pOn                = trace_port(ports[port_id++]);
            sOdp.pThreshold         = trace_port(ports[port_id++]);
            sOdp.pKnee              = trace_port(ports[port_id++]);
            sOdp.pResonance         = trace_port(ports[port_id++]);
            sOdp.pCurveMesh         = trace_port(ports[port_id++]);
            sClip.pOn               = trace_port(ports[port_id++]);
            sClip.pFunction         = trace_port(ports[port_id++]);
            sClip.pThreshold        = trace_port(ports[port_id++]);
            sClip.pPumping          = trace_port(ports[port_id++]);
            sClip.pCurveMesh        = trace_port(ports[port_id++]);

            lsp_trace("Skipping graph visibility ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                trace_port(ports[port_id++]); // Skip input level graph visibility
                trace_port(ports[port_id++]); // Skip output level graph visibility
                trace_port(ports[port_id++]); // Skip gain reduction graph visibility
            }

            lsp_trace("Binding analysis ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->pFftInSwitch         = trace_port(ports[port_id++]);
                c->pFftOutSwitch        = trace_port(ports[port_id++]);
                c->pFftInMesh           = trace_port(ports[port_id++]);
                c->pFftOutMesh          = trace_port(ports[port_id++]);
            }

            lsp_trace("Binding band metering ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];

                    b->pIn                  = trace_port(ports[port_id++]);
                    b->pOut                 = trace_port(ports[port_id++]);
                    b->pRed                 = trace_port(ports[port_id++]);

                    b->pOdpIn               = trace_port(ports[port_id++]);
                    b->pOdpOut              = trace_port(ports[port_id++]);
                    b->pOdpRed              = trace_port(ports[port_id++]);

                    b->pClipIn              = trace_port(ports[port_id++]);
                    b->pClipOut             = trace_port(ports[port_id++]);
                    b->pClipRed             = trace_port(ports[port_id++]);

                    b->pTimeMesh            = trace_port(ports[port_id++]);
                }
            }

            // Bind channel metering
            lsp_trace("Binding channel metering ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->pIn                  = trace_port(ports[port_id++]);
                c->pOut                 = trace_port(ports[port_id++]);
                c->pRed                 = trace_port(ports[port_id++]);

                c->pOdpIn               = trace_port(ports[port_id++]);
                c->pOdpOut              = trace_port(ports[port_id++]);
                c->pOdpRed              = trace_port(ports[port_id++]);

                c->pClipIn              = trace_port(ports[port_id++]);
                c->pClipOut             = trace_port(ports[port_id++]);
                c->pClipRed             = trace_port(ports[port_id++]);

                c->pTimeMesh            = trace_port(ports[port_id++]);
            }

            // Initialize curve (logarithmic) in range of -72 .. +24 db
            float delta = (meta::clipper::ODP_CURVE_DB_MAX - meta::clipper::ODP_CURVE_DB_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vOdp[i]         = dspu::db_to_gain(meta::clipper::ODP_CURVE_DB_MIN + delta * i);

            delta       = (meta::clipper::CLIP_CURVE_DB_MAX - meta::clipper::CLIP_CURVE_DB_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vLogSigmoid[i]  = dspu::db_to_gain(meta::clipper::CLIP_CURVE_DB_MIN + delta * i);

            delta       = (meta::clipper::CLIP_CURVE_X_MAX - meta::clipper::CLIP_CURVE_X_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vLinSigmoid[i]  = meta::clipper::CLIP_CURVE_X_MIN + delta * i;

            delta       = meta::clipper::TIME_HISTORY_MAX / (meta::clipper::TIME_MESH_POINTS - 1);
            for (size_t i=0; i<meta::clipper::TIME_MESH_POINTS; ++i)
                vTime[i]    = meta::clipper::TIME_HISTORY_MAX - i*delta;
        }

        void clipper::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void clipper::do_destroy()
        {
            // Destroy channels
            // TODO: check that all objects have been destroyed properly
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    c->sBypass.destroy();
                    c->sDryDelay.destroy();
                    c->sScDelay.destroy();
                    c->sSc.destroy();
                    c->sEqualizer.destroy();
                    c->sIIRXOver.destroy();
                    c->sFFTXOver.destroy();
                    c->sInGraph.destroy();
                    c->sOutGraph.destroy();

                    for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                    {
                        band_t *b       = &c->vBands[j];

                        b->sSc.destroy();
                        b->sScDelay.destroy();
                        b->sInDelay.destroy();
                        b->sPreDelay.destroy();
                        b->sPostDelay.destroy();
                        b->sInGraph.destroy();
                        b->sOutGraph.destroy();
                    }
                }
                vChannels   = NULL;
            }

            // Destroy analyzer
            sAnalyzer.destroy();
            sCounter.destroy();

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
            const size_t fft_rank           = select_fft_rank(sr);
            const size_t max_delay_fft      = (1 << fft_rank);
            const size_t max_odp_delay      = (
                dspu::hz_to_samples(sr, meta::clipper::ODP_REACT1_MIN) * 0.5f +
                dspu::hz_to_samples(sr, meta::clipper::ODP_REACT2_MIN) * 0.5f +
                dspu::hz_to_samples(sr, meta::clipper::ODP_REACT3_MIN) * 0.5f +
                dspu::hz_to_samples(sr, meta::clipper::ODP_REACT4_MIN) * 0.5f);
            const size_t max_global_delay   = dspu::millis_to_samples(sr, meta::clipper::ODP_REACT1_MAX) * 0.5f;
            const size_t samples_per_dot    = dspu::seconds_to_samples(
                sr, meta::clipper::TIME_HISTORY_MAX / meta::clipper::TIME_MESH_POINTS);

            sCounter.set_sample_rate(sr, true);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.init(sr);
                c->sDryDelay.init(max_delay_fft + max_odp_delay + max_global_delay);
                c->sScDelay.init(max_global_delay);
                c->sSc.init(1, meta::clipper::ODP_REACT_MAX);
                c->sSc.set_sample_rate(sr);
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
                c->sInGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                c->sOutGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];

                    b->sSc.init(1, 1000.0f / meta::clipper::ODP_REACT1_MIN);
                    b->sSc.set_sample_rate(sr);
                    b->sScDelay.init(max_odp_delay);
                    b->sInDelay.init(max_odp_delay);
                    b->sPreDelay.init(max_odp_delay);
                    b->sPostDelay.init(max_odp_delay);
                    b->sInGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                    b->sOutGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                }
            }

            // Commit sample rate to analyzer
            sAnalyzer.init(
                nChannels * 2,
                meta::clipper::FFT_RANK,
                MAX_SAMPLE_RATE,
                meta::clipper::REFRESH_RATE,
                max_delay_fft + max_odp_delay);
            sAnalyzer.set_rank(meta::clipper::FFT_RANK);
            sAnalyzer.set_envelope(dspu::envelope::WHITE_NOISE);
            sAnalyzer.set_window(meta::clipper::FFT_WINDOW);
            sAnalyzer.set_rate(meta::clipper::REFRESH_RATE);
            sAnalyzer.set_sample_rate(sr);

            if (sAnalyzer.needs_reconfiguration())
            {
                for (size_t i=0; i < meta::clipper::BANDS_MAX; ++i)
                {
                    processor_t *p          = &vProc[i];
                    p->nFlags              |= PF_DIRTY_BAND | PF_SYNC_BAND;
                }
            }
        }

        inline size_t clipper::filter_slope(size_t slope)
        {
            return slope;
        }

        inline float clipper::fft_filter_slope(size_t slope)
        {
            return -24.0f * slope;
        }

        bool clipper::update_odp_params(odp_params_t *params)
        {
            const float threshold   = dspu::db_to_gain(params->pThreshold->value());
            const float knee        = dspu::db_to_gain(params->pKnee->value());

            if ((threshold == params->fThreshold) &&
                (knee == params->fKnee))
                return false;

            params->fThreshold      = threshold;
            params->fKnee           = knee;

            return true;
        }

        bool clipper::update_clip_params(clip_params_t *params)
        {
            dspu::sigmoid::function_t func = vSigmoidFunctions[size_t(params->pFunction->value())];
            const float threshold   = lsp_min(params->pThreshold->value(), 0.99f);
            const float pumping     = dspu::db_to_gain(params->pPumping->value());

            if ((func == params->pFunc) &&
                (threshold == params->fThreshold) &&
                (pumping == params->fPumping))
                return false;

            params->pFunc           = func;
            params->fThreshold      = threshold;
            params->fPumping        = pumping;
            params->fKnee           = 1.0f - threshold;
            params->fScaling        = 1.0f / params->fKnee;

            return true;
        }

        void clipper::calc_odp_compressor(compressor_t *c, const odp_params_t *params)
        {
            const float th  = params->fThreshold;
            const float kn  = params->fKnee;

            c->x0           = th;
            c->x1           = th / kn;
            c->x2           = th * kn;

            float y1        = c->x1;
            float y2        = th;
            float dy        = y2 - y1;
            float dx1       = 1.0f/(c->x2 - c->x1);
            float dx2       = dx1*dx1;

            float k         = 1.0f;

            c->c            = k;
            c->b            = (3.0 * dy) * dx2 - (2.0 * k)*dx1;
            c->a            = (k - (2.0*dy)*dx1)*dx2;
        }

        float clipper::odp_curve(const compressor_t *c, float x)
        {
            if (x >= c->x2)
                return c->x0;
            if (x <= c->x1)
                return x;

            float v    = x - c->x1;
            return ((v * c->a + c->b) * v + c->c)*v + c->x1;
        }

        float clipper::odp_gain(const compressor_t *c, float x)
        {
            if (x >= c->x2)
                return c->x0 / x;
            if (x <= c->x1)
                return 1.0f;

            float v    = x - c->x1;
            return (((v * c->a + c->b) * v + c->c)*v + c->x1)/x;
        }

        void clipper::odp_curve(float *dst, const float *x, const compressor_t *c, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]      = odp_curve(c, x[i]);
        }

        void clipper::odp_gain(float *dst, const float *x, const compressor_t *c, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]      = odp_gain(c, x[i]);
        }

        void clipper::odp_link(float *dst, const float *src, float link, size_t count)
        {
            const float rlink = 1.0f - link;
            for (size_t i=0; i<count; ++i)
            {
                const float k = rlink + src[i] * link;
                dst[i]  = dst[i] * k;
            }
        }

        float clipper::clip_curve(const clip_params_t *p, float x)
        {
            float s = x * p->fPumping;
            if (s > p->fThreshold)
            {
                s               = (s - p->fThreshold) * p->fScaling;
                return p->pFunc(s) * p->fKnee + p->fThreshold;
            }
            else if (s < -p->fThreshold)
            {
                s               = (s + p->fThreshold) * p->fScaling;
                return p->pFunc(s) * p->fKnee - p->fThreshold;
            }

            return s;
        }

        void clipper::clip_curve(float *dst, const float *x, const clip_params_t *p, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]      = clip_curve(p, x[i]);
        }

        void clipper::update_settings()
        {
            bool bypass             = pBypass->value() >= 0.5f;
            fThresh                 = dspu::db_to_gain(-pThresh->value());
            float out_gain          = pGainOut->value();
            size_t active_channels  = 0;
            bool sync_band_curves   = false;

            fInGain                 = pGainIn->value() * fThresh;
            fOutGain                = (pBoosting->value() >= 0.5f) ? out_gain : out_gain / fThresh;

            xover_mode_t mode       = (pXOverMode->value() >= 1) ? XOVER_FFT : XOVER_IIR;
            if (mode != enXOverMode)
            {
                enXOverMode             = mode;
                sync_band_curves        = true;
            }

            // Configure split frequencies
            const size_t num_splits = (pExtraBandOn->value() >= 0.5f) ? meta::clipper::BANDS_MAX-1 : meta::clipper::BANDS_MAX-2;
            for (size_t i=0; i<meta::clipper::BANDS_MAX-1; ++i)
            {
                split_t *sp             = &vSplits[i];
                sp->fFreq               = sp->pFreq->value();
                sp->fOdpLink            = sp->pOdpLink->value();
            }

            // Check if we have solo option for bands and ansysis optios for channels
            bool has_solo   = false;
            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                processor_t *p          = &vProc[j];
                p->nFlags              &= uint32_t(~PF_ENABLED);
                p->fPreamp              = dspu::db_to_gain(p->pPreamp->value());
                p->fMakeup              = dspu::db_to_gain(p->pMakeup->value());
                if ((p->pSolo->value() >= 0.5f) && (j<=num_splits))
                    has_solo                = true;
            }

            for (size_t j=0; j<=num_splits; ++j)
            {
                processor_t *p          = &vProc[j];
                bool mute               = (p->pMute->value() >= 0.5f) || ((has_solo) && (p->pSolo->value() < 0.5f));
                p->nFlags               = lsp_setflag(p->nFlags, PF_ENABLED, !mute);
            }

            // Configure crossover
            size_t xover_latency    = 0;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->sBypass.set_bypass(bypass);

                if (enXOverMode == XOVER_IIR)
                {
                    dspu::Crossover *xc     = &c->sIIRXOver;
                    const size_t iir_slope  = filter_slope(pXOverSlope->value() + 1);
                    const size_t hpf_slope  = filter_slope(pHpfSlope->value());
                    const size_t lpf_slope  = filter_slope(pLpfSlope->value());

                    // Configure split points for crossover
                    for (size_t j=0; j<meta::clipper::BANDS_MAX-1; ++j)
                    {
                        split_t *sp         = &vSplits[j];

                        xc->set_frequency(j, sp->fFreq);
                        xc->set_mode(j, dspu::CROSS_MODE_BT);
                        xc->set_slope(j, (j < num_splits) ? iir_slope : 0);
                    }

                    // Check if we need to synchronize state of bands
                    if (xc->needs_reconfiguration())
                        sync_band_curves        = true;

                    // Configure equalizer
                    dspu::filter_params_t fp;
                    fp.nType    = (hpf_slope > 0) ? dspu::FLT_BT_LRX_HIPASS : dspu::FLT_NONE;
                    fp.nSlope   = hpf_slope;
                    fp.fFreq    = pHpfFreq->value();
                    fp.fFreq2   = fp.fFreq;
                    fp.fGain    = 1.0f;
                    fp.fQuality = 0.0f;

                    c->sEqualizer.set_params(0, &fp);

                    fp.nType    = (lpf_slope > 0) ? dspu::FLT_BT_LRX_LOPASS : dspu::FLT_NONE;
                    fp.nSlope   = lpf_slope;
                    fp.fFreq    = pLpfFreq->value();
                    fp.fFreq2   = fp.fFreq;

                    c->sEqualizer.set_params(1, &fp);

                    // Obtain new equalizer curve if needed
                    if ((i == 0) && (c->sEqualizer.configuration_changed()))
                    {
                        for (size_t offset=0; offset<meta::clipper::FFT_MESH_POINTS; )
                        {
                            size_t count    = lsp_min(BUFFER_SIZE >> 1, meta::clipper::FFT_MESH_POINTS - offset);
                            c->sEqualizer.freq_chart(vBuffer, &vFreqs[offset], count);
                            dsp::pcomplex_mod(&vTrEq[offset], vBuffer, count);
                            offset         += count;
                        }

                        sync_band_curves    = true;
                    }
                }
                else // (enXOverMode == XOVER_FFT)
                {
                    dspu::FFTCrossover *xf  = &c->sFFTXOver;
                    const float fft_slope   = fft_filter_slope(pXOverSlope->value() + 1);
                    const float hpf_slope   = fft_filter_slope(pHpfSlope->value());
                    const float lpf_slope   = fft_filter_slope(pLpfSlope->value());

                    // Configure split points for crossover
                    for (size_t j=0; j < meta::clipper::BANDS_MAX; ++j)
                    {
//                        band_t *b       = &c->vBands[i];

                        if (j > 0)
                        {
                            xf->enable_hpf(j, true);
                            xf->set_hpf_frequency(j, vSplits[j-1].fFreq);
                            xf->set_hpf_slope(j, fft_slope);
                        }
                        else
                        {
                            xf->enable_hpf(j, hpf_slope < -1.0f);
                            xf->set_hpf_frequency(j, pHpfFreq->value());
                            xf->set_hpf_slope(j, hpf_slope);
                        }

                        if (j < num_splits)
                        {
                            xf->enable_lpf(j, true);
                            xf->set_lpf_frequency(j, vSplits[j].fFreq);
                            xf->set_lpf_slope(j, fft_slope);
                        }
                        else
                        {
                            xf->enable_lpf(j, lpf_slope < -1.0f);
                            xf->set_lpf_frequency(j, pLpfFreq->value());
                            xf->set_lpf_slope(j, lpf_slope);
                        }

                        xf->enable_band(j, j <= num_splits);

                        lsp_trace("hpf[%d] = %f", int(j), xf->hpf_frequency(j));
                        lsp_trace("lpf[%d] = %f", int(j), xf->lpf_frequency(j));
                        lsp_trace("on[%d]  = %s", int(j), xf->band_enabled(j) ? "true" : "false");
                    }

                    // Check if we need to synchronize state of bands
                    if (xf->needs_update())
                        sync_band_curves        = true;

                    xover_latency     = lsp_max(xover_latency, xf->latency());
                }
            }

            // Configure clipping
            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                processor_t *p          = &vProc[j];

                p->fStereoLink          = (p->pStereoLink != NULL) ? p->pStereoLink->value() * 0.01f : 1.0f;
                p->nFlags               = lsp_setflag(p->nFlags, PF_ODP_ENABLED, p->sOdp.pOn->value() >= 0.5f);
                if (update_odp_params(&p->sOdp))
                {
                    calc_odp_compressor(&p->sComp, &p->sOdp);
                    p->nFlags              |= PF_SYNC_ODP;
                }
                p->nFlags               = lsp_setflag(p->nFlags, PF_CLIP_ENABLED, p->sClip.pOn->value() >= 0.5f);
                if (update_clip_params(&p->sClip))
                    p->nFlags              |= PF_SYNC_CLIP;
            }

            fStereoLink             = (pStereoLink != NULL) ? pStereoLink->value() * 0.01f : 1.0f;
            nFlags                  = lsp_setflag(nFlags, GF_OUT_CLIP, pOutClipperOn->value() >= 0.5f);
            nFlags                  = lsp_setflag(nFlags, GF_ODP_ENABLED, sOdp.pOn->value() >= 0.5f);
            if (update_odp_params(&sOdp))
            {
                calc_odp_compressor(&sComp, &sOdp);
                nFlags                 |= GF_SYNC_ODP;
            }
            nFlags                  = lsp_setflag(nFlags, GF_CLIP_ENABLED, sClip.pOn->value() >= 0.5f);
            if (update_clip_params(&sClip))
                nFlags                 |= GF_SYNC_CLIP;

            // Configure analyzer
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->nFlags               = lsp_setflag(c->nFlags, CF_IN_FFT, c->pFftInSwitch->value() >= 0.5f);
                c->nFlags               = lsp_setflag(c->nFlags, CF_OUT_FFT, c->pFftOutSwitch->value() >= 0.5f);
            }

            sAnalyzer.set_reactivity(pFftReactivity->value());
            sAnalyzer.set_shift(pFftShift->value() * 100.0f);
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                sAnalyzer.enable_channel(c->nAnInChannel, c->nFlags & CF_IN_FFT);
                sAnalyzer.enable_channel(c->nAnOutChannel, c->nFlags & CF_OUT_FFT);
                if (c->nFlags & (CF_IN_FFT | CF_OUT_FFT))
                    ++active_channels;

                sAnalyzer.set_channel_delay(c->nAnInChannel, xover_latency);
            }
            sAnalyzer.set_activity(active_channels > 0);

            if (sAnalyzer.needs_reconfiguration())
            {
                sAnalyzer.reconfigure();
                sAnalyzer.get_frequencies(vFreqs, vIndexes, SPEC_FREQ_MIN, SPEC_FREQ_MAX, meta::clipper::FFT_MESH_POINTS);
            }

            // Mark crossover bands out of sync
            if (sync_band_curves)
            {
                for (size_t i=0; i < meta::clipper::BANDS_MAX; ++i)
                {
                    processor_t *b          = &vProc[i];
                    b->nFlags              |= PF_DIRTY_BAND | PF_SYNC_BAND;
                }
            }

            // Adjust the compensation delays
            size_t max_band_latency = 0;
            size_t clip_latency     = dspu::millis_to_samples(fSampleRate, sOdp.pResonance->value()) * 0.5f;
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Update sidechain reactivity
                c->sSc.set_reactivity(sOdp.pResonance->value());
                c->sSc.set_mode(dspu::SCM_RMS);
                c->sSc.set_stereo_mode(dspu::SCSM_STEREO);

                c->sScDelay.set_delay(clip_latency);

                // Adjust RMS compensation delay for each band
                size_t band_latency     = 0;
                for (size_t j=0; j < meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];
                    processor_t *p          = &vProc[j];
                    size_t sc_latency       = dspu::hz_to_samples(fSampleRate, p->sOdp.pResonance->value()) * 0.5f;

                    b->sSc.set_reactivity(1000.0f / p->sOdp.pResonance->value());
                    b->sSc.set_mode(dspu::SCM_RMS);
                    b->sSc.set_stereo_mode(dspu::SCSM_STEREO);

                    b->sScDelay.set_delay(sc_latency);
                    b->sInDelay.set_delay(sc_latency);
                    b->sPreDelay.set_delay(band_latency);

                    band_latency           += sc_latency;
                }

                max_band_latency        = lsp_max(max_band_latency, band_latency);
            }

            // Compute the final latency
            size_t latency = xover_latency + max_band_latency + clip_latency;
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                for (size_t j=0; j < meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b       = &c->vBands[j];
                    b->sPostDelay.set_delay(max_band_latency - b->sPreDelay.delay() - b->sInDelay.delay());
                }
                c->sDryDelay.set_delay(latency);
            }
            set_latency(latency);

            // Debug
        #ifdef LSP_TRACE
            lsp_trace("max band latency = %d", int(max_band_latency));
            lsp_trace("clip latency = %d", int(clip_latency));
            lsp_trace("latency = %d", int(latency));

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                lsp_trace("CHANNEL %d:", int(i));

                // Adjust RMS compensation delay for each band
                for (size_t j=0; j < meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];
                    lsp_trace("  BAND %d:", int(j));

                    lsp_trace("    pre_delay  = %d", int(b->sPreDelay.delay()));
                    lsp_trace("    sc_delay   = %d", int(b->sScDelay.delay()));
                    lsp_trace("    in_delay   = %d", int(b->sInDelay.delay()));
                    lsp_trace("    post_delay = %d", int(b->sPostDelay.delay()));
                }
            }
        #endif /* LSP_TRACE */
        }

        void clipper::process_band(void *object, void *subject, size_t band, const float *data, size_t sample, size_t count)
        {
            clipper *self           = static_cast<clipper *>(object);
            channel_t *c            = static_cast<channel_t *>(subject);
            band_t *b               = &c->vBands[band];
            processor_t *p          = &self->vProc[band];

            // Copy data to the data buffer and apply pre-amplification gain
            dsp::mul_k3(&b->vData[sample], data, p->fPreamp, count);
        }

        void clipper::bind_input_buffers()
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                c->vIn              = c->pDataIn->buffer<float>();
                c->vOut             = c->pDataOut->buffer<float>();

                c->fIn              = GAIN_AMP_M_INF_DB;
                c->fOut             = GAIN_AMP_M_INF_DB;
                c->fRed             = GAIN_AMP_P_72_DB;

                c->fOdpIn           = GAIN_AMP_M_INF_DB;
                c->fOdpOut          = GAIN_AMP_M_INF_DB;
                c->fOdpRed          = GAIN_AMP_P_72_DB;

                c->fClipIn          = GAIN_AMP_M_INF_DB;
                c->fClipOut         = GAIN_AMP_M_INF_DB;
                c->fClipRed         = GAIN_AMP_P_72_DB;

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b           = &c->vBands[j];

                    b->fIn              = GAIN_AMP_M_INF_DB;
                    b->fOut             = GAIN_AMP_M_INF_DB;
                    b->fRed             = GAIN_AMP_P_72_DB;

                    b->fOdpIn           = GAIN_AMP_M_INF_DB;
                    b->fOdpOut          = GAIN_AMP_M_INF_DB;
                    b->fOdpRed          = GAIN_AMP_P_72_DB;

                    b->fClipIn          = GAIN_AMP_M_INF_DB;
                    b->fClipOut         = GAIN_AMP_M_INF_DB;
                    b->fClipRed         = GAIN_AMP_P_72_DB;
                }
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

        void clipper::split_bands(size_t samples)
        {
            if (enXOverMode == XOVER_IIR)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];

                    // Apply input gain and split signal into multiple bands
                    dsp::mul_k3(c->vInAnalyze, c->vIn, fInGain, samples);
                    c->sEqualizer.process(vBuffer, c->vInAnalyze, samples);
                    c->sIIRXOver.process(vBuffer, samples);
                }
            }
            else // (enXOverMode == XOVER_FFT)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];

                    // Apply input gain and split signal into multiple bands
                    dsp::mul_k3(c->vInAnalyze, c->vIn, fInGain, samples);
                    c->sFFTXOver.process(c->vInAnalyze, samples);
                }
            }

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];
                    processor_t *p          = &vProc[j];
                    if (!(p->nFlags & PF_ENABLED))
                        dsp::fill_zero(b->vData, samples);
                }
            }
        }

        void clipper::process_bands(size_t samples)
        {
            if (nChannels > 1)
            {
                // Stereo version
                channel_t *l            = &vChannels[0];
                channel_t *r            = &vChannels[1];

                for (size_t i=0; i<meta::clipper::BANDS_MAX; ++i)
                {
                    band_t *lb              = &l->vBands[i];
                    band_t *rb              = &r->vBands[i];
                    processor_t *p          = &vProc[i];

                    // Apply pre-delay and overdrive protection link
                    if (i > 0)
                    {
                        // Apply phase compensation
                        lb->sPreDelay.process(lb->vData, lb->vData, samples);
                        rb->sPreDelay.process(rb->vData, rb->vData, samples);

                        // Remember input data for analysis
                        lb->sInDelay.process(lb->vInData, lb->vData, samples);
                        rb->sInDelay.process(rb->vInData, rb->vData, samples);

                        // Perform linking with previous band
                        const float odp_linking = vSplits[i-1].fOdpLink;
                        if (odp_linking > 0.0f)
                        {
                            odp_link(lb->vData, l->vSc, odp_linking, samples);
                            odp_link(rb->vData, r->vSc, odp_linking, samples);
                        }
                    }
                    else
                    {
                        // Remember input data for analysis
                        lb->sInDelay.process(lb->vInData, lb->vData, samples);
                        rb->sInDelay.process(rb->vInData, rb->vData, samples);
                    }

                    // Measure signal at the input of the band
                    const size_t idx_in_l   = dsp::abs_max_index(lb->vInData, samples);
                    const size_t idx_in_r   = dsp::abs_max_index(rb->vInData, samples);
                    const float in_l        = fabsf(lb->vInData[idx_in_l]);
                    const float in_r        = fabsf(rb->vInData[idx_in_r]);
                    lb->sInGraph.process(lb->vInData, samples);
                    rb->sInGraph.process(rb->vInData, samples);

                    // Process sidechain signal
                    if (p->fStereoLink >= 1.0f)
                    {
                        dsp::lr_to_mid(r->vSc, lb->vData, rb->vData, samples);
                        lb->sSc.process(l->vSc, const_cast<const float **>(&r->vSc), samples);
                        rb->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                    }
                    else if (p->fStereoLink > 0.0f)
                    {
                        dsp::mix_copy2(l->vSc, lb->vData, rb->vData, 1.0f - p->fStereoLink * 0.5f, p->fStereoLink * 0.5f, samples);
                        dsp::mix_copy2(r->vSc, lb->vData, rb->vData, p->fStereoLink * 0.5f, 1.0f - p->fStereoLink * 0.5f, samples);

                        lb->sSc.process(l->vSc, const_cast<const float **>(&l->vSc), samples);
                        rb->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                    }
                    else
                    {
                        lb->sSc.process(l->vSc, const_cast<const float **>(&lb->vData), samples);
                        rb->sSc.process(r->vSc, const_cast<const float **>(&rb->vData), samples);
                    }
                    lb->sScDelay.process(lb->vData, lb->vData, samples);
                    rb->sScDelay.process(rb->vData, rb->vData, samples);

                    // Overdrive protection
                    if (p->nFlags & PF_ODP_ENABLED)
                    {
                        // Measure input signal
                        const size_t odp_idx_l  = dsp::abs_max_index(l->vSc, samples);
                        const size_t odp_idx_r  = dsp::abs_max_index(r->vSc, samples);
                        const float odp_in_l    = l->vSc[odp_idx_l];
                        const float odp_in_r    = r->vSc[odp_idx_r];

                        // Apply ODP
                        odp_gain(l->vSc, l->vSc, &p->sComp, samples);
                        odp_gain(r->vSc, r->vSc, &p->sComp, samples);
                        dsp::mul2(lb->vData, l->vSc, samples);
                        dsp::mul2(rb->vData, r->vSc, samples);

                        // Measure output
                        const float odp_red_l   = l->vSc[odp_idx_l];
                        const float odp_red_r   = r->vSc[odp_idx_r];
                        const float odp_out_l   = odp_in_l * odp_red_l;
                        const float odp_out_r   = odp_in_r * odp_red_r;

                        lb->fOdpIn              = lsp_max(lb->fOdpIn, odp_in_l);
                        lb->fOdpOut             = lsp_max(lb->fOdpOut, odp_out_l);
                        lb->fOdpRed             = lsp_min(lb->fOdpRed, odp_red_l);

                        rb->fOdpIn              = lsp_max(rb->fOdpIn, odp_in_r);
                        rb->fOdpOut             = lsp_max(rb->fOdpOut, odp_out_r);
                        rb->fOdpRed             = lsp_min(rb->fOdpRed, odp_red_r);
                    }
                    else
                    {
                        dsp::fill_one(l->vSc, samples);
                        dsp::fill_one(r->vSc, samples);

                        lb->fOdpIn              = GAIN_AMP_M_INF_DB;
                        lb->fOdpOut             = GAIN_AMP_M_INF_DB;
                        lb->fOdpRed             = GAIN_AMP_0_DB;

                        rb->fOdpIn              = GAIN_AMP_M_INF_DB;
                        rb->fOdpOut             = GAIN_AMP_M_INF_DB;
                        rb->fOdpRed             = GAIN_AMP_0_DB;
                    }

                    // Clipping
                    if (p->nFlags & PF_CLIP_ENABLED)
                    {
                        // Mesure input
                        const size_t clip_idx_l = dsp::abs_max_index(lb->vData, samples);
                        const size_t clip_idx_r = dsp::abs_max_index(rb->vData, samples);
                        const float clip_in_l   = fabsf(lb->vData[clip_idx_l]);
                        const float clip_in_r   = fabsf(rb->vData[clip_idx_r]);

                        // Do clipping
                        clip_curve(lb->vData, lb->vData, &p->sClip, samples);
                        clip_curve(rb->vData, rb->vData, &p->sClip, samples);

                        // Measure output
                        const float clip_out_l  = fabsf(lb->vData[clip_idx_l]);
                        const float clip_out_r  = fabsf(rb->vData[clip_idx_r]);
                        const float clip_red_l  = (clip_in_l >= GAIN_AMP_M_120_DB) ? clip_out_l / clip_in_l : GAIN_AMP_0_DB;
                        const float clip_red_r  = (clip_in_r >= GAIN_AMP_M_120_DB) ? clip_out_r / clip_in_r : GAIN_AMP_0_DB;

                        lb->fClipIn             = lsp_max(lb->fClipIn, clip_in_l);
                        lb->fClipOut            = lsp_max(lb->fClipOut, clip_out_l);
                        lb->fClipRed            = lsp_min(lb->fClipRed, clip_red_l);

                        rb->fClipIn             = lsp_max(rb->fClipIn, clip_in_r);
                        rb->fClipOut            = lsp_max(rb->fClipOut, clip_out_r);
                        rb->fClipRed            = lsp_min(rb->fClipRed, clip_red_r);
                    }
                    else
                    {
                        lb->fClipIn             = GAIN_AMP_M_INF_DB;
                        lb->fClipOut            = GAIN_AMP_M_INF_DB;
                        lb->fClipRed            = GAIN_AMP_0_DB;

                        rb->fClipIn             = GAIN_AMP_M_INF_DB;
                        rb->fClipOut            = GAIN_AMP_M_INF_DB;
                        rb->fClipRed            = GAIN_AMP_0_DB;
                    }

                    // Perform output metering
                    const float out_l       = fabsf(lb->vData[idx_in_l]) * p->fMakeup;
                    const float out_r       = fabsf(rb->vData[idx_in_r]) * p->fMakeup;
                    const float red_l       = (in_l >= GAIN_AMP_M_120_DB) ? out_l / in_l : GAIN_AMP_0_DB;
                    const float red_r       = (in_r >= GAIN_AMP_M_120_DB) ? out_r / in_r : GAIN_AMP_0_DB;
                    lb->sOutGraph.process(lb->vData, p->fMakeup, samples);
                    rb->sOutGraph.process(rb->vData, p->fMakeup, samples);

                    lb->fIn                 = lsp_max(lb->fIn, in_l);
                    lb->fOut                = lsp_max(lb->fOut, out_l);
                    lb->fRed                = lsp_min(lb->fRed, red_l);

                    rb->fIn                 = lsp_max(rb->fIn, in_r);
                    rb->fOut                = lsp_max(rb->fOut, out_r);
                    rb->fRed                = lsp_min(rb->fRed, red_r);
                }
            }
            else
            {
                // TODO: implement mono version of algorithm
            }
        }

        void clipper::process_output_clipper(size_t samples)
        {
            if (nChannels > 1)
            {
                // Stereo version
                channel_t *l            = &vChannels[0];
                channel_t *r            = &vChannels[1];

                // Process sidechain signal
                if (nFlags & GF_OUT_CLIP)
                {
                    if (fStereoLink >= 1.0f)
                    {
                        dsp::lr_to_mid(r->vSc, l->vData, r->vData, samples);
                        l->sSc.process(l->vSc, const_cast<const float **>(&r->vSc), samples);
                        r->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                    }
                    else if (fStereoLink > 0.0f)
                    {
                        dsp::mix_copy2(l->vSc, l->vData, r->vData, 1.0f - fStereoLink * 0.5f, fStereoLink * 0.5f, samples);
                        dsp::mix_copy2(r->vSc, l->vData, r->vData, fStereoLink * 0.5f, 1.0f - fStereoLink * 0.5f, samples);

                        l->sSc.process(l->vSc, const_cast<const float **>(&l->vSc), samples);
                        r->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                    }
                    else
                    {
                        l->sSc.process(l->vSc, const_cast<const float **>(&l->vData), samples);
                        r->sSc.process(r->vSc, const_cast<const float **>(&r->vData), samples);
                    }
                }
                l->sScDelay.process(l->vData, l->vData, samples);
                r->sScDelay.process(r->vData, r->vData, samples);

                // Measure signal at the input of the band
                const size_t idx_in_l   = dsp::abs_max_index(l->vData, samples);
                const size_t idx_in_r   = dsp::abs_max_index(r->vData, samples);
                const float in_l        = fabsf(l->vData[idx_in_l]);
                const float in_r        = fabsf(r->vData[idx_in_r]);
                l->sInGraph.process(l->vData, samples);
                r->sInGraph.process(r->vData, samples);

                // Overdrive protection
                if ((nFlags & (GF_ODP_ENABLED | GF_OUT_CLIP)) == (GF_ODP_ENABLED | GF_OUT_CLIP))
                {
                    // Measure input signal
                    const size_t odp_idx_l  = dsp::abs_max_index(l->vSc, samples);
                    const size_t odp_idx_r  = dsp::abs_max_index(r->vSc, samples);
                    const float odp_in_l    = l->vSc[odp_idx_l];
                    const float odp_in_r    = r->vSc[odp_idx_r];

                    // Apply ODP
                    odp_gain(l->vSc, l->vSc, &sComp, samples);
                    odp_gain(r->vSc, r->vSc, &sComp, samples);
                    dsp::mul2(l->vData, l->vSc, samples);
                    dsp::mul2(r->vData, r->vSc, samples);

                    // Measure output
                    const float odp_red_l   = l->vSc[odp_idx_l];
                    const float odp_red_r   = r->vSc[odp_idx_r];
                    const float odp_out_l   = odp_in_l * odp_red_l;
                    const float odp_out_r   = odp_in_r * odp_red_r;

                    l->fOdpIn               = lsp_max(l->fOdpIn, odp_in_l);
                    l->fOdpOut              = lsp_max(l->fOdpOut, odp_out_l);
                    l->fOdpRed              = lsp_min(l->fOdpRed, odp_red_l);

                    r->fOdpIn               = lsp_max(r->fOdpIn, odp_in_r);
                    r->fOdpOut              = lsp_max(r->fOdpOut, odp_out_r);
                    r->fOdpRed              = lsp_min(r->fOdpRed, odp_red_r);
                }
                else
                {
                    dsp::fill_one(l->vSc, samples);
                    dsp::fill_one(r->vSc, samples);

                    l->fOdpIn               = GAIN_AMP_M_INF_DB;
                    l->fOdpOut              = GAIN_AMP_M_INF_DB;
                    l->fOdpRed              = GAIN_AMP_0_DB;

                    r->fOdpIn               = GAIN_AMP_M_INF_DB;
                    r->fOdpOut              = GAIN_AMP_M_INF_DB;
                    r->fOdpRed              = GAIN_AMP_0_DB;
                }

                // Clipping
                if ((nFlags & (GF_CLIP_ENABLED | GF_OUT_CLIP)) == (GF_CLIP_ENABLED | GF_OUT_CLIP))
                {
                    // Mesure input
                    const size_t clip_idx_l = dsp::abs_max_index(l->vData, samples);
                    const size_t clip_idx_r = dsp::abs_max_index(r->vData, samples);
                    const float clip_in_l   = fabsf(l->vData[clip_idx_l]);
                    const float clip_in_r   = fabsf(r->vData[clip_idx_r]);

                    // Do clipping
                    clip_curve(l->vData, l->vData, &sClip, samples);
                    clip_curve(r->vData, r->vData, &sClip, samples);

                    // Measure output
                    const float clip_out_l  = fabsf(l->vData[clip_idx_l]);
                    const float clip_out_r  = fabsf(r->vData[clip_idx_r]);
                    const float clip_red_l  = (clip_in_l >= GAIN_AMP_M_120_DB) ? clip_out_l / clip_in_l : GAIN_AMP_0_DB;
                    const float clip_red_r  = (clip_in_r >= GAIN_AMP_M_120_DB) ? clip_out_r / clip_in_r : GAIN_AMP_0_DB;

                    l->fClipIn              = lsp_max(l->fClipIn, clip_in_l);
                    l->fClipOut             = lsp_max(l->fClipOut, clip_out_l);
                    l->fClipRed             = lsp_min(l->fClipRed, clip_red_l);

                    r->fClipIn              = lsp_max(r->fClipIn, clip_in_r);
                    r->fClipOut             = lsp_max(r->fClipOut, clip_out_r);
                    r->fClipRed             = lsp_min(r->fClipRed, clip_red_r);
                }
                else
                {
                    l->fClipIn              = GAIN_AMP_M_INF_DB;
                    l->fClipOut             = GAIN_AMP_M_INF_DB;
                    l->fClipRed             = GAIN_AMP_0_DB;

                    r->fClipIn              = GAIN_AMP_M_INF_DB;
                    r->fClipOut             = GAIN_AMP_M_INF_DB;
                    r->fClipRed             = GAIN_AMP_0_DB;
                }

                // Perform output metering
                const float out_l       = fabsf(l->vData[idx_in_l]);
                const float out_r       = fabsf(r->vData[idx_in_r]);
                const float red_l       = (in_l >= GAIN_AMP_M_120_DB) ? out_l / in_l : GAIN_AMP_0_DB;
                const float red_r       = (in_r >= GAIN_AMP_M_120_DB) ? out_r / in_r : GAIN_AMP_0_DB;
                l->sOutGraph.process(l->vData, samples);
                r->sOutGraph.process(r->vData, samples);

                l->fIn                  = lsp_max(l->fIn, in_l);
                l->fOut                 = lsp_max(l->fOut, out_l);
                l->fRed                 = lsp_min(l->fRed, red_l);

                r->fIn                  = lsp_max(r->fIn, in_r);
                r->fOut                 = lsp_max(r->fOut, out_r);
                r->fRed                 = lsp_min(r->fRed, red_r);
            }
            else
            {
                // TODO: implement mono version of algorithm
            }
        }

        void clipper::merge_bands(size_t samples)
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                size_t merged = 0;

                // Mix all bands together
                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b               = &c->vBands[j];
                    processor_t *p          = &vProc[j];
                    if (!(p->nFlags & PF_ENABLED))
                    {
                        b->sPostDelay.append(b->vData, samples); // Apply post-delay
                        continue;
                    }

                    if (merged++)
                        b->sPostDelay.process_add(c->vData, b->vData, p->fMakeup, samples); // Apply post-delay
                    else
                        b->sPostDelay.process(c->vData, b->vData, p->fMakeup, samples); // Apply post-delay
                }

                // Fill output with zero if there is no data at output
                if (!merged)
                    dsp::fill_zero(c->vData, samples);
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

        void clipper::output_meters()
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];

                c->pIn->set_value(c->fIn);
                c->pOut->set_value(c->fOut);
                c->pRed->set_value(c->fRed);

                c->pOdpIn->set_value(c->fOdpIn);
                c->pOdpOut->set_value(c->fOdpOut);
                c->pOdpRed->set_value(c->fOdpRed);

                c->pClipIn->set_value(c->fClipIn);
                c->pClipOut->set_value(c->fClipOut);
                c->pClipRed->set_value(c->fClipRed);

                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b       = &c->vBands[j];

                    b->pIn->set_value(b->fIn);
                    b->pOut->set_value(b->fOut);
                    b->pRed->set_value(b->fRed);

                    b->pOdpIn->set_value(b->fOdpIn);
                    b->pOdpOut->set_value(b->fOdpOut);
                    b->pOdpRed->set_value(b->fOdpRed);

                    b->pClipIn->set_value(b->fClipIn);
                    b->pClipOut->set_value(b->fClipOut);
                    b->pClipRed->set_value(b->fClipRed);
                }
            }
        }

        void clipper::output_mesh_curves(size_t samples)
        {
            plug::mesh_t *mesh  = NULL;

            // Update processor curves if needed
            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                channel_t *c        = &vChannels[0];
                processor_t *p      = &vProc[j];
                if (p->nFlags & PF_DIRTY_BAND)
                {
                    if (enXOverMode == XOVER_IIR)
                    {
                        for (size_t offset=0; offset<meta::clipper::FFT_MESH_POINTS; )
                        {
                            size_t count    = lsp_min(BUFFER_SIZE >> 1, meta::clipper::FFT_MESH_POINTS - offset);

                            c->sIIRXOver.freq_chart(j, vBuffer, &vFreqs[offset], count);
                            dsp::pcomplex_mod(vBuffer, vBuffer, count);
                            dsp::mul3(&p->vTr[offset], &vTrEq[offset], vBuffer, count);

                            offset         += count;
                        }
                    }
                    else // (enXOverMode == XOVER_FFT)
                    {
                        for (size_t offset=0; offset<meta::clipper::FFT_MESH_POINTS; )
                        {
                            size_t count    = lsp_min(BUFFER_SIZE, meta::clipper::FFT_MESH_POINTS - offset);
                            c->sFFTXOver.freq_chart(j, &p->vTr[offset], &vFreqs[offset], count);
                            offset         += count;
                        }
                    }

                    p->nFlags      &= uint32_t(~PF_DIRTY_BAND);
                }

                // Sync band filter curve
                if (p->nFlags & PF_SYNC_BAND)
                {
                    mesh                = (p->pFreqChart != NULL) ? p->pFreqChart->buffer<plug::mesh_t>() : NULL;
                    if ((mesh != NULL) && (mesh->isEmpty()))
                    {
                        // Add extra points
                        mesh->pvData[0][0] = SPEC_FREQ_MIN*0.5f;
                        mesh->pvData[0][meta::clipper::FFT_MESH_POINTS+1] = SPEC_FREQ_MAX * 2.0f;
                        mesh->pvData[1][0] = 0.0f;
                        mesh->pvData[1][meta::clipper::FFT_MESH_POINTS+1] = 0.0f;

                        // Fill mesh
                        dsp::copy(&mesh->pvData[0][1], vFreqs, meta::clipper::FFT_MESH_POINTS);
                        dsp::copy(&mesh->pvData[1][1], p->vTr, meta::clipper::FFT_MESH_POINTS);
                        mesh->data(2, meta::clipper::FFT_MESH_POINTS + 2);

                        // Mark mesh as synchronized
                        p->nFlags      &= uint32_t(~PF_SYNC_BAND);
                    }
                }

                // Sync ODP curve
                if (p->nFlags & PF_SYNC_ODP)
                {
                    mesh                = (p->sOdp.pCurveMesh != NULL) ? p->sOdp.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                    if ((mesh != NULL) && (mesh->isEmpty()))
                    {
                        dsp::copy(mesh->pvData[0], vOdp, meta::clipper::CURVE_MESH_POINTS);
                        odp_curve(mesh->pvData[1], vOdp, &p->sComp, meta::clipper::CURVE_MESH_POINTS);
                        mesh->data(2, meta::clipper::CURVE_MESH_POINTS);

                        // Mark mesh as synchronized
                        p->nFlags      &= uint32_t(~PF_SYNC_ODP);
                    }
                }

                // Sync sigmoid curve
                if (p->nFlags & PF_SYNC_CLIP)
                {
                    mesh                = (p->sClip.pCurveMesh != NULL) ? p->sClip.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                    if ((mesh != NULL) && (mesh->isEmpty()))
                    {
                        dsp::copy(mesh->pvData[0], vLinSigmoid, meta::clipper::CURVE_MESH_POINTS);
                        clip_curve(mesh->pvData[1], vLinSigmoid, &p->sClip, meta::clipper::CURVE_MESH_POINTS);
                        dsp::copy(mesh->pvData[2], vLogSigmoid, meta::clipper::CURVE_MESH_POINTS);
                        clip_curve(mesh->pvData[3], vLogSigmoid, &p->sClip, meta::clipper::CURVE_MESH_POINTS);
                        mesh->data(4, meta::clipper::CURVE_MESH_POINTS);

                        // Mark mesh as synchronized
                        p->nFlags      &= uint32_t(~PF_SYNC_CLIP);
                    }
                }
            }

            // Sync ODP curve
            if (nFlags & GF_SYNC_ODP)
            {
                mesh                = (sOdp.pCurveMesh != NULL) ? sOdp.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    dsp::copy(mesh->pvData[0], vOdp, meta::clipper::CURVE_MESH_POINTS);
                    odp_curve(mesh->pvData[1], vOdp, &sComp, meta::clipper::CURVE_MESH_POINTS);
                    mesh->data(2, meta::clipper::CURVE_MESH_POINTS);

                    // Mark mesh as synchronized
                    nFlags             &= uint32_t(~GF_SYNC_ODP);
                }
            }

            // Sync sigmoid curve
            if (nFlags & GF_SYNC_CLIP)
            {
                mesh                = (sClip.pCurveMesh != NULL) ? sClip.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    dsp::copy(mesh->pvData[0], vLinSigmoid, meta::clipper::CURVE_MESH_POINTS);
                    clip_curve(mesh->pvData[1], vLinSigmoid, &sClip, meta::clipper::CURVE_MESH_POINTS);
                    dsp::copy(mesh->pvData[2], vLogSigmoid, meta::clipper::CURVE_MESH_POINTS);
                    clip_curve(mesh->pvData[3], vLogSigmoid, &sClip, meta::clipper::CURVE_MESH_POINTS);
                    mesh->data(4, meta::clipper::CURVE_MESH_POINTS);

                    // Mark mesh as synchronized
                    nFlags             &= uint32_t(~GF_SYNC_CLIP);
                }
            }

            // Output FFT mesh data for each channel
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                // Output FFT curve for input
                mesh            = (c->pFftInMesh != NULL) ? c->pFftInMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    if (c->nFlags & CF_IN_FFT)
                    {
                        // Add extra points
                        mesh->pvData[0][0] = SPEC_FREQ_MIN * 0.5f;
                        mesh->pvData[0][meta::clipper::FFT_MESH_POINTS+1] = SPEC_FREQ_MAX * 2.0f;
                        mesh->pvData[1][0] = 0.0f;
                        mesh->pvData[1][meta::clipper::FFT_MESH_POINTS+1] = 0.0f;

                        // Copy frequency points
                        dsp::copy(&mesh->pvData[0][1], vFreqs, meta::clipper::FFT_MESH_POINTS);
                        sAnalyzer.get_spectrum(c->nAnInChannel, &mesh->pvData[1][1], vIndexes, meta::clipper::FFT_MESH_POINTS);

                        // Mark mesh containing data
                        mesh->data(2, meta::clipper::FFT_MESH_POINTS + 2);
                    }
                    else
                        mesh->data(2, 0);
                }

                // Output FFT curve for output
                mesh            = (c->pFftOutMesh != NULL) ? c->pFftOutMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    if (sAnalyzer.channel_active(c->nAnOutChannel))
                    {
                        // Copy frequency points
                        dsp::copy(mesh->pvData[0], vFreqs, meta::clipper::FFT_MESH_POINTS);
                        sAnalyzer.get_spectrum(c->nAnOutChannel, mesh->pvData[1], vIndexes, meta::clipper::FFT_MESH_POINTS);

                        // Mark mesh containing data
                        mesh->data(2, meta::clipper::FFT_MESH_POINTS);
                    }
                    else
                        mesh->data(2, 0);
                }

                // Output oscilloscope graphs for output clipper
                plug::mesh_t *mesh    = c->pTimeMesh->buffer<plug::mesh_t>();
                if (nFlags & GF_OUT_CLIP)
                {
                    // Fill time values
                    float *t        = mesh->pvData[0];
                    float *in       = mesh->pvData[1];
                    float *out      = mesh->pvData[2];
                    float *red      = mesh->pvData[3];

                    dsp::copy(&t[2], vTime, meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&in[2], c->sInGraph.data(), meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&out[2], c->sOutGraph.data(), meta::clipper::TIME_MESH_POINTS);

                    for (size_t k=2; k<meta::clipper::TIME_MESH_POINTS + 2; ++k)
                        red[k]      = lsp_max(out[k], GAIN_AMP_M_120_DB) / lsp_max(in[k], GAIN_AMP_M_120_DB);

                    // Generate extra points
                    t[0]            = t[2] + meta::clipper::TIME_HISTORY_GAP;
                    t[1]            = t[0];
                    in[0]           = 0.0f;
                    in[1]           = in[2];
                    out[0]          = out[2];
                    out[1]          = out[2];
                    red[0]          = red[2];
                    red[1]          = red[2];

                    t              += meta::clipper::TIME_MESH_POINTS + 2;
                    in             += meta::clipper::TIME_MESH_POINTS + 2;
                    out            += meta::clipper::TIME_MESH_POINTS + 2;
                    red            += meta::clipper::TIME_MESH_POINTS + 2;

                    t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
                    t[1]            = t[0];
                    in[0]           = in[-1];
                    in[1]           = 0.0f;
                    out[0]          = out[-1];
                    out[1]          = out[-1];
                    red[0]          = red[-1];
                    red[1]          = red[-1];

                    // Notify mesh contains data
                    mesh->data(4, meta::clipper::TIME_MESH_POINTS + 4);
                }
                else
                    mesh->data(4, 0);

                // Output oscilloscope graphs for band
                for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
                {
                    band_t *b           = &c->vBands[j];
                    processor_t *p      = &vProc[j];

                    // Output metering mesh data
                    plug::mesh_t *mesh    = b->pTimeMesh->buffer<plug::mesh_t>();
                    if ((mesh != NULL) && (mesh->isEmpty()))
                    {
                        if (p->nFlags & PF_ENABLED)
                        {
                            // Fill time values
                            float *t        = mesh->pvData[0];
                            float *in       = mesh->pvData[1];
                            float *out      = mesh->pvData[2];
                            float *red      = mesh->pvData[3];

                            dsp::copy(&t[2], vTime, meta::clipper::TIME_MESH_POINTS);
                            dsp::copy(&in[2], b->sInGraph.data(), meta::clipper::TIME_MESH_POINTS);
                            dsp::copy(&out[2], b->sOutGraph.data(), meta::clipper::TIME_MESH_POINTS);

                            for (size_t k=2; k<meta::clipper::TIME_MESH_POINTS + 2; ++k)
                                red[k]      = lsp_max(out[k], GAIN_AMP_M_120_DB) / lsp_max(in[k], GAIN_AMP_M_120_DB);

                            // Generate extra points
                            t[0]            = t[2] + meta::clipper::TIME_HISTORY_GAP;
                            t[1]            = t[0];
                            in[0]           = 0.0f;
                            in[1]           = in[2];
                            out[0]          = out[2];
                            out[1]          = out[2];
                            red[0]          = red[2];
                            red[1]          = red[2];

                            t              += meta::clipper::TIME_MESH_POINTS + 2;
                            in             += meta::clipper::TIME_MESH_POINTS + 2;
                            out            += meta::clipper::TIME_MESH_POINTS + 2;
                            red            += meta::clipper::TIME_MESH_POINTS + 2;

                            t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
                            t[1]            = t[0];
                            in[0]           = in[-1];
                            in[1]           = 0.0f;
                            out[0]          = out[-1];
                            out[1]          = out[-1];
                            red[0]          = red[-1];
                            red[1]          = red[-1];

                            // Notify mesh contains data
                            mesh->data(4, meta::clipper::TIME_MESH_POINTS + 4);
                        }
                        else
                            mesh->data(4, 0);
                    }
                }
            }

            // Update counter
            sCounter.submit(samples);
        }

        void clipper::output_signal(size_t samples)
        {
            // Process the signal
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                dsp::mul_k2(c->vData, fOutGain, samples);
                c->sDryDelay.process(vBuffer, c->vIn, samples);
                c->sBypass.process(c->vOut, vBuffer, c->vData, samples);
            }
        }

        void clipper::process(size_t samples)
        {
            bind_input_buffers();

            for (size_t offset = 0; offset < samples; )
            {
                size_t to_do    = lsp_min(samples - offset, BUFFER_SIZE);

                split_bands(to_do);
                process_bands(to_do);
                merge_bands(to_do);
                process_output_clipper(to_do);
                perform_analysis(to_do);
                output_signal(to_do);

                advance_buffers(to_do);
                offset         += to_do;
            }

            output_meters();
            output_mesh_curves(samples);
        }

        void clipper::ui_activated()
        {
            // Force meshes to become synchronized with UI
            for (size_t j=0; j<meta::clipper::BANDS_MAX; ++j)
            {
                processor_t *p      = &vProc[j];
                p->nFlags          |= PF_SYNC_ALL;
            }
            nFlags             |= GF_SYNC_ALL;
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

                    v->write("pDataIn", c->pDataIn);
                    v->write("pDataOut", c->pDataOut);
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


