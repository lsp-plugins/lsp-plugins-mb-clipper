/*
 * Copyright (C) 2023 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2023 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-mb-clipper
 * Created on: 11 ноя 2023 г.
 *
 * lsp-plugins-mb-clipper is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-mb-clipper is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-mb-clipper. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PRIVATE_META_CLIPPER_H_
#define PRIVATE_META_CLIPPER_H_

#include <lsp-plug.in/plug-fw/meta/types.h>
#include <lsp-plug.in/plug-fw/const.h>

#include <lsp-plug.in/dsp-units/misc/windows.h>

namespace lsp
{
    //-------------------------------------------------------------------------
    // Plugin metadata
    namespace meta
    {
        typedef struct mb_clipper
        {
            static constexpr size_t BANDS_MAX               = 4;
            static constexpr size_t FFT_MESH_POINTS         = 640;
            static constexpr size_t FFT_XOVER_RANK_MIN      = 12;
            static constexpr size_t FFT_XOVER_FREQ_MIN      = 44100;
            static constexpr size_t FFT_RANK                = 13;
            static constexpr size_t FFT_ITEMS               = 1 << FFT_RANK;
            static constexpr size_t FFT_WINDOW              = dspu::windows::HANN;
            static constexpr float  ODP_CURVE_DB_MIN        = -18.0f;
            static constexpr float  ODP_CURVE_DB_MAX        = 6.0f;
            static constexpr float  CLIP_CURVE_DB_MIN       = -36.0f;
            static constexpr float  CLIP_CURVE_DB_MAX       = 12.0f;
            static constexpr float  CLIP_CURVE_X_MIN        = -0.25f;
            static constexpr float  CLIP_CURVE_X_MAX        = 2.25f;
            static constexpr size_t CURVE_MESH_POINTS       = 256;
            static constexpr size_t REFRESH_RATE            = 20;
            static constexpr size_t TIME_MESH_POINTS        = 400;
            static constexpr float  TIME_HISTORY_MAX        = 5.0f;
            static constexpr float  TIME_HISTORY_GAP        = 0.5f;
            static constexpr float  WAVEFORM_HISTORY_MAX    = 1.0f;
            static constexpr float  LUFS_MEASUREMENT_PERIOD = 400.0f;
            static constexpr float  LUFS_LIMITER_REACT      = 48.0f;

            static constexpr float  THRESHOLD_MIN           = -48.0f;
            static constexpr float  THRESHOLD_MAX           = 0.0f;
            static constexpr float  THRESHOLD_DFL           = 0.0f;
            static constexpr float  THRESHOLD_STEP          = 0.01f;

            static constexpr float  DCOFF_MIN               = -100.0f;
            static constexpr float  DCOFF_MAX               = 100.0f;
            static constexpr float  DCOFF_DFL               = 0.0f;
            static constexpr float  DCOFF_STEP              = 0.05f;

            static constexpr float  REACT_TIME_MIN          = 0.000f;
            static constexpr float  REACT_TIME_MAX          = 1.000f;
            static constexpr float  REACT_TIME_DFL          = 0.200f;
            static constexpr float  REACT_TIME_STEP         = 0.003f;

            static constexpr float  ZOOM_MIN                = GAIN_AMP_M_18_DB;
            static constexpr float  ZOOM_MAX                = GAIN_AMP_0_DB;
            static constexpr float  ZOOM_DFL                = GAIN_AMP_0_DB;
            static constexpr float  ZOOM_STEP               = 0.0125f;

            static constexpr float  SPLIT1_MIN              = 20.0f;
            static constexpr float  SPLIT1_MAX              = 250.0f;
            static constexpr float  SPLIT1_DFL              = 125.0f;
            static constexpr float  SPLIT1_STEP             = 0.0004f;

            static constexpr float  SPLIT2_MIN              = SPLIT1_MAX + 25.0f;
            static constexpr float  SPLIT2_MAX              = 5000.0f;
            static constexpr float  SPLIT2_DFL              = 4500.0f;
            static constexpr float  SPLIT2_STEP             = 0.0004f;

            static constexpr float  SPLIT3_MIN              = SPLIT2_MAX + 250.0f;
            static constexpr float  SPLIT3_MAX              = 14000.0f;
            static constexpr float  SPLIT3_DFL              = 7500.0f;
            static constexpr float  SPLIT3_STEP             = 0.0003f;

            static constexpr float  HPF_FREQ_MIN            = 10.0f;
            static constexpr float  HPF_FREQ_MAX            = 60.0f;
            static constexpr float  HPF_FREQ_DFL            = 30.0f;
            static constexpr float  HPF_FREQ_STEP           = 0.0003f;

            static constexpr float  LPF_FREQ_MIN            = 10000.0f;
            static constexpr float  LPF_FREQ_MAX            = 20000.0f;
            static constexpr float  LPF_FREQ_DFL            = 12500.0f;
            static constexpr float  LPF_FREQ_STEP           = 0.0001f;

            static constexpr float  STEREO_LINK_MIN         = 0.0f;
            static constexpr float  STEREO_LINK_MAX         = 100.0f;
            static constexpr float  STEREO_LINK_DFL         = 50.0f;
            static constexpr float  STEREO_LINK_STEP        = 0.01f;

            static constexpr float  MAKEUP_MIN              = -24.0f;
            static constexpr float  MAKEUP_MAX              = 24.0f;
            static constexpr float  MAKEUP_DFL              = 0.0f;
            static constexpr float  MAKEUP_STEP             = 0.005f;

            static constexpr float  PREAMP_MIN              = -24.0f;
            static constexpr float  PREAMP_MAX              = 24.0f;
            static constexpr float  PREAMP_DFL              = 0.0f;
            static constexpr float  PREAMP_STEP             = 0.008f;

            static constexpr float  LUFS_THRESH_MIN         = -36.0f;
            static constexpr float  LUFS_THRESH_MAX         = 0.0f;
            static constexpr float  LUFS_THRESH_DFL         = -3.0f;
            static constexpr float  LUFS_THRESH_STEP        = 0.01f;

            static constexpr float  ODP_LINK_MIN            = GAIN_AMP_M_INF_DB;
            static constexpr float  ODP_LINK_MAX            = GAIN_AMP_0_DB;
            static constexpr float  ODP_LINK_DFL            = GAIN_AMP_M_6_DB;
            static constexpr float  ODP_LINK_STEP           = 0.02f;

            static constexpr float  ODP_KNEE_MIN            = 0.0f;
            static constexpr float  ODP_KNEE_MAX            = 6.0f;
            static constexpr float  ODP_KNEE_DFL            = 1.5f;
            static constexpr float  ODP_KNEE_STEP           = 0.001f;

            static constexpr float  ODP_THRESHOLD_MIN       = -12.0f;
            static constexpr float  ODP_THRESHOLD_MAX       = 0.0f;
            static constexpr float  ODP_THRESHOLD_DFL       = -1.5f;
            static constexpr float  ODP_THRESHOLD_STEP      = 0.005f;

            static constexpr float  ODP_REACT_MIN           = 1.0f;
            static constexpr float  ODP_REACT_MAX           = 200.0f;
            static constexpr float  ODP_REACT_DFL           = 50.0f;
            static constexpr float  ODP_REACT_STEP          = 0.005f;

            static constexpr float  ODP_REACT1_MIN          = SPEC_FREQ_MIN;
            static constexpr float  ODP_REACT1_MAX          = SPLIT1_MAX;
            static constexpr float  ODP_REACT1_DFL          = 50.0f;
            static constexpr float  ODP_REACT1_STEP         = 0.00025f;

            static constexpr float  ODP_REACT2_MIN          = SPLIT1_MIN;
            static constexpr float  ODP_REACT2_MAX          = SPLIT2_MAX;
            static constexpr float  ODP_REACT2_DFL          = 220.0f;
            static constexpr float  ODP_REACT2_STEP         = 0.00025f;

            static constexpr float  ODP_REACT3_MIN          = SPLIT2_MIN;
            static constexpr float  ODP_REACT3_MAX          = SPLIT3_MAX;
            static constexpr float  ODP_REACT3_DFL          = 5000.0f;
            static constexpr float  ODP_REACT3_STEP         = 0.00025f;

            static constexpr float  ODP_REACT4_MIN          = SPLIT3_MIN;
            static constexpr float  ODP_REACT4_MAX          = SPEC_FREQ_MAX;
            static constexpr float  ODP_REACT4_DFL          = 8000.0f;
            static constexpr float  ODP_REACT4_STEP         = 0.00025f;

            static constexpr float  CLIP_THRESHOLD_MIN      = GAIN_AMP_M_INF_DB;
            static constexpr float  CLIP_THRESHOLD_MAX      = GAIN_AMP_0_DB;
            static constexpr float  CLIP_THRESHOLD_DFL      = GAIN_AMP_M_3_DB;
            static constexpr float  CLIP_THRESHOLD_STEP     = 0.005f;

            static constexpr float  CLIP_PUMPING_MIN        = -12.0f;
            static constexpr float  CLIP_PUMPING_MAX        = 12.0f;
            static constexpr float  CLIP_PUMPING_DFL        = 0.0f;
            static constexpr float  CLIP_PUMPING_STEP       = 0.005f;

            enum dithering_t
            {
                DITHER_NONE,
                DITHER_7BIT,
                DITHER_8BIT,
                DITHER_11BIT,
                DITHER_12BIT,
                DITHER_15BIT,
                DITHER_16BIT,
                DITHER_23BIT,
                DITHER_24BIT,

                DITHER_DEFAULT  = DITHER_NONE
            };
        } mb_clipper;

        // Plugin type metadata
        extern const plugin_t mb_clipper_mono;
        extern const plugin_t mb_clipper_stereo;

    } /* namespace meta */
} /* namespace lsp */

#endif /* PRIVATE_META_CLIPPER_H_ */
