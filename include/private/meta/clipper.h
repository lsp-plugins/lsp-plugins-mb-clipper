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
        typedef struct clipper
        {
            static constexpr size_t BANDS_MAX               = 4;
            static constexpr size_t FFT_MESH_POINTS         = 640;
            static constexpr size_t FFT_XOVER_RANK_MIN      = 12;
            static constexpr size_t FFT_XOVER_FREQ_MIN      = 44100;
            static constexpr size_t FFT_RANK                = 13;
            static constexpr size_t FFT_ITEMS               = 1 << FFT_RANK;
            static constexpr size_t FFT_WINDOW              = dspu::windows::HANN;
            static constexpr size_t REFRESH_RATE            = 20;

            static constexpr float  THRESHOLD_MIN           = -48.0f;
            static constexpr float  THRESHOLD_MAX           = 0.0f;
            static constexpr float  THRESHOLD_DFL           = 0.0f;
            static constexpr float  THRESHOLD_STEP          = 0.05f;

            static constexpr float  REACT_TIME_MIN          = 0.000f;
            static constexpr float  REACT_TIME_MAX          = 1.000f;
            static constexpr float  REACT_TIME_DFL          = 0.200f;
            static constexpr float  REACT_TIME_STEP         = 0.001f;

            static constexpr float  ZOOM_MIN                = GAIN_AMP_M_18_DB;
            static constexpr float  ZOOM_MAX                = GAIN_AMP_0_DB;
            static constexpr float  ZOOM_DFL                = GAIN_AMP_0_DB;
            static constexpr float  ZOOM_STEP               = 0.0125f;

            static constexpr float  SPLIT1_MIN              = 20.0f;
            static constexpr float  SPLIT1_MAX              = 250.0f;
            static constexpr float  SPLIT1_DFL              = 125.0f;
            static constexpr float  SPLIT1_STEP             = 0.0005f;

            static constexpr float  SPLIT2_MIN              = SPLIT1_MAX + 25;
            static constexpr float  SPLIT2_MAX              = 5000.0f;
            static constexpr float  SPLIT2_DFL              = 1500.0f;
            static constexpr float  SPLIT2_STEP             = SPLIT1_STEP;

            static constexpr float  SPLIT3_MIN              = SPLIT2_MAX + 250;
            static constexpr float  SPLIT3_MAX              = 14000.0f;
            static constexpr float  SPLIT3_DFL              = 7500.0f;
            static constexpr float  SPLIT3_STEP             = SPLIT1_STEP;

            static constexpr float  HPF_FREQ_MIN            = 10.0f;
            static constexpr float  HPF_FREQ_MAX            = 60.0f;
            static constexpr float  HPF_FREQ_DFL            = 10.0f;
            static constexpr float  HPF_FREQ_STEP           = 0.0025f;

            static constexpr float  LPF_FREQ_MIN            = 10000.0f;
            static constexpr float  LPF_FREQ_MAX            = 20000.0f;
            static constexpr float  LPF_FREQ_DFL            = 20000.0f;
            static constexpr float  LPF_FREQ_STEP           = 0.0025f;

            static constexpr float  STEREO_LINK_MIN         = 0.0f;
            static constexpr float  STEREO_LINK_MAX         = 100.0f;
            static constexpr float  STEREO_LINK_DFL         = 50.0f;
            static constexpr float  STEREO_LINK_STEP        = 0.05f;

            static constexpr float  RMS_TIME_MIN            = 0.01f;
            static constexpr float  RMS_TIME_MAX            = 50.0f;
            static constexpr float  RMS_TIME_DFL            = 10.0f;
            static constexpr float  RMS_TIME_STEP           = 0.01f;

        } clipper;

        // Plugin type metadata
        extern const plugin_t clipper_mono;
        extern const plugin_t clipper_stereo;

    } /* namespace meta */
} /* namespace lsp */

#endif /* PRIVATE_META_CLIPPER_H_ */
