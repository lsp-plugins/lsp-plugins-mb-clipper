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

#ifndef PRIVATE_PLUGINS_CLIPPER_H_
#define PRIVATE_PLUGINS_CLIPPER_H_

#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/ctl/Counter.h>
#include <lsp-plug.in/dsp-units/filters/Equalizer.h>
#include <lsp-plug.in/dsp-units/misc/sigmoid.h>
#include <lsp-plug.in/dsp-units/util/Analyzer.h>
#include <lsp-plug.in/dsp-units/util/Crossover.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/util/FFTCrossover.h>
#include <lsp-plug.in/dsp-units/util/Sidechain.h>
#include <lsp-plug.in/plug-fw/plug.h>

#include <private/meta/clipper.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Base class for the latency compensation delay
         */
        class clipper: public plug::Module
        {
            protected:
                enum xover_mode_t
                {
                    XOVER_IIR,
                    XOVER_FFT
                };

                enum band_flags_t
                {
                    PF_ENABLED          = 1 << 0,           // Band is enabled
                    PF_ODP_ENABLED      = 1 << 1,           // Overdrive protection enabled
                    PF_SIGMOID_ENABLED  = 1 << 2,           // Sigmoid processing enabled
                    PF_DIRTY_BAND       = 1 << 3,           // Update band filter curve
                    PF_SYNC_BAND        = 1 << 4,           // Sync band filter curve
                    PF_SYNC_ODP         = 1 << 5,           // Sync overdrive protection curve
                    PF_SYNC_CLIP        = 1 << 6,           // Sync sigmoid clipping curve

                    PF_SYNC_ALL         = PF_SYNC_BAND | PF_SYNC_ODP | PF_SYNC_CLIP
                };

                enum channel_flags_t
                {
                    CF_IN_FFT           = 1 << 0,           // Input FFT analysis is enabled
                    CF_OUT_FFT          = 1 << 1,           // Output FFT analysis is enabled
                };

                typedef struct compressor_t
                {
                    float       x0, x1, x2;
                    float       t, g;
                    float       a, b, c;
                } compressor_t;

                // Overdrive protection module
                typedef struct odp_params_t
                {
                    float               fThreshold;         // Threshold
                    float               fKnee;              // Knee
                    float               fMakeup;            // Makeup gain

                    plug::IPort        *pOn;                // Enable overdrive protection
                    plug::IPort        *pThreshold;         // Threshold
                    plug::IPort        *pKnee;              // Knee
                    plug::IPort        *pMakeup;            // Makeup gain
                    plug::IPort        *pResonance;         // Resonance frequency
                    plug::IPort        *pCurveMesh;         // Curve chart mesh
                } odp_params_t;

                typedef struct clip_params_t
                {
                    dspu::sigmoid::function_t   pFunc;      // Sigmoid function
                    float               fThreshold;         // Threshold
                    float               fPumping;           // Pumping
                    float               fScaling;           // Sigmoid scaling
                    float               fKnee;              // Knee

                    plug::IPort        *pOn;                // Enable sigmoid function
                    plug::IPort        *pFunction;          // Sigmoid function
                    plug::IPort        *pThreshold;         // Sigmoid threshold
                    plug::IPort        *pPumping;           // Sigmoid pumping
                    plug::IPort        *pCurveMesh;         // Curve chart mesh
                } clip_params_t;

                typedef struct band_t
                {
                    dspu::Sidechain     sSc;                // Sidechain
                    dspu::Delay         sScDelay;           // Sidechain latency compensation delay
                    dspu::Delay         sPreDelay;          // Signal pre-delay
                    dspu::Delay         sPostDelay;         // Signal post-delay

                    float              *vData;              // Data buffer

                    float               fOdpIn;             // Overdrive protection input level
                    float               fOdpOut;            // Overdrive protection out level
                    float               fOdpRed;            // Overdrive protection reduction level

                    float               fSigmoidIn;         // Sigmoid input level measured
                    float               fSigmoidOut;        // Sigmoid output level measured
                    float               fSigmoidRed;        // Sigmoid reduction level measured

                    plug::IPort        *pOdpIn;             // Input level meter
                    plug::IPort        *pOdpOut;            // Output level meter
                    plug::IPort        *pOdpRed;            // Reduction level meter

                    plug::IPort        *pClipIn;            // Clipping input level meter
                    plug::IPort        *pClipOut;           // Clipping output level meter
                    plug::IPort        *pClipRed;           // Clipping reduction level meter
                } band_t;

                typedef struct processor_t
                {
                    compressor_t        sComp;              // Simple compressor
                    odp_params_t        sOdp;               // Overdrive protection params
                    clip_params_t       sClip;              // Clipping parameters

                    uint32_t            nFlags;             // Processor flags
                    float               fStereoLink;        // Stereo link

                    float              *vTr;                // Transfer function

                    plug::IPort        *pSolo;              // Solo button
                    plug::IPort        *pMute;              // Mute button
                    plug::IPort        *pFreqChart;         // Frequency chart
                    plug::IPort        *pStereoLink;        // Stereo link
                } processor_t;

                typedef struct split_t
                {
                    float               fFreq;              // Split frequency
                    float               fOdpLink;           // Overdrive protection link
                    plug::IPort        *pFreq;              // Split frequency
                    plug::IPort        *pOdpLink;           // Overdrive protection link
                } split_t;

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass        sBypass;            // Bypass
                    dspu::Delay         sDryDelay;          // Delay for the dry signal
                    dspu::Equalizer     sEqualizer;         // Equalizer for cut-off low/high frequencies
                    dspu::Crossover     sIIRXOver;          // IIR crossover
                    dspu::FFTCrossover  sFFTXOver;          // FFT crossover
                    band_t              vBands[meta::clipper::BANDS_MAX];   // Bands for processing

                    uint32_t            nAnInChannel;       // Analyzer input channel
                    uint32_t            nAnOutChannel;      // Analyzer output channel
                    uint32_t            nFlags;             // Flags
                    float               fInGain;            // Input level meter
                    float               fOutGain;           // Output level meter

                    // Buffers
                    float              *vIn;                // Input buffer
                    float              *vOut;               // Output buffer
                    float              *vData;              // Data buffer
                    float              *vSc;                // Sidechain buffer
                    float              *vInAnalyze;         // Input data analysis

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port
                    plug::IPort        *pFftInSwitch;       // Input FFT enable switch
                    plug::IPort        *pFftOutSwitch;      // Output FFT enable switch
                    plug::IPort        *pFftInMesh;         // Input FFT mesh
                    plug::IPort        *pFftOutMesh;        // Output FFT mesh
                } channel_t;

                static dspu::sigmoid::function_t    vSigmoidFunctions[];

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels

                dspu::Analyzer      sAnalyzer;          // FFT analyzer
                dspu::Counter       sCounter;           // Counter
                split_t             vSplits[meta::clipper::BANDS_MAX-1];
                processor_t         vProc[meta::clipper::BANDS_MAX];      // Processor

                xover_mode_t        enXOverMode;        // Crossover mode
                float               fInGain;            // Input gain
                float               fThresh;            // Threshold
                float               fOutGain;           // Output gain

                float              *vBuffer;            // Temporary buffer
                float              *vFreqs;             // FFT frequencies
                uint32_t           *vIndexes;           // Analyzer FFT indexes
                float              *vTrEq;              // Equalizer transfer function (real values)
                float              *vOdp;               // Overdrive protection curve input gain values
                float              *vLinSigmoid;        // Linear scale for sigmoid
                float              *vLogSigmoid;        // Logarithmic scale for sigmoid

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pGainIn;            // Input gain
                plug::IPort        *pGainOut;           // Output gain
                plug::IPort        *pThresh;            // Threshold
                plug::IPort        *pBoosting;          // Boosting mode
                plug::IPort        *pXOverMode;         // Crossover mode
                plug::IPort        *pXOverSlope;        // Crossover filter slope
                plug::IPort        *pFftReactivity;     // FFT reactivity
                plug::IPort        *pFftShift;          // FFT shift gain
                plug::IPort        *pZoom;              // Zoom
                plug::IPort        *pHpfSlope;          // HPF filter slope
                plug::IPort        *pHpfFreq;           // HPF filter frequency
                plug::IPort        *pLpfSlope;          // LPF filter slope
                plug::IPort        *pLpfFreq;           // LPF filter frequency
                plug::IPort        *pExtraBandOn;       // Enable extra band
                plug::IPort        *pFilterCurves;      // Band filter curves

                uint8_t            *pData;              // Allocated data

            protected:
                static size_t           select_fft_rank(size_t sample_rate);
                static void             process_band(void *object, void *subject, size_t band, const float *data, size_t sample, size_t count);
                static inline size_t    filter_slope(size_t slope);
                static inline float     fft_filter_slope(size_t slope);

                static bool             update_odp_params(odp_params_t *params);
                static bool             update_clip_params(clip_params_t *params);

                static void             calc_odp_compressor(compressor_t *c, const odp_params_t *params);
                static inline float     odp_curve(const compressor_t *c, float x);
                static inline float     odp_gain(const compressor_t *c, float x);
                static void             odp_curve(float *dst, const float *x, const compressor_t *c, size_t count);
                static void             odp_gain(float *dst, const float *x, const compressor_t *c, size_t count);
                static void             odp_link(float *dst, const float *src, float link, size_t count);

                static float            clip_curve(const clip_params_t *p, float x);
                static float            clip_gain(const clip_params_t *p, float x);
                static void             clip_curve(float *dst, const float *x, const clip_params_t *p, size_t count);
                static void             clip_gain(float *dst, const float *x, const clip_params_t *p, size_t count);

            protected:
                void                    do_destroy();
                void                    bind_input_buffers();
                void                    split_bands(size_t samples);
                void                    process_bands(size_t samples);
                void                    perform_analysis(size_t samples);
                void                    output_signal(size_t samples);
                void                    advance_buffers(size_t samples);
                void                    merge_bands(size_t samples);
                void                    output_mesh_curves(size_t samples);

            public:
                explicit clipper(const meta::plugin_t *meta);
                clipper (const clipper &) = delete;
                clipper (clipper &&) = delete;
                virtual ~clipper() override;

                clipper & operator = (const clipper &) = delete;
                clipper & operator = (clipper &&) = delete;

                virtual void            init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void            destroy() override;

            public:
                virtual void            update_sample_rate(long sr) override;
                virtual void            update_settings() override;
                virtual void            process(size_t samples) override;
                virtual void            ui_activated() override;
                virtual bool            inline_display(plug::ICanvas *cv, size_t width, size_t height) override;
                virtual void            dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_CLIPPER_H_ */

