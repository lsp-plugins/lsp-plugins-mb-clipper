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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/clipper.h>

#define LSP_PLUGINS_CLIPPER_VERSION_MAJOR       1
#define LSP_PLUGINS_CLIPPER_VERSION_MINOR       0
#define LSP_PLUGINS_CLIPPER_VERSION_MICRO       0

#define LSP_PLUGINS_CLIPPER_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_CLIPPER_VERSION_MAJOR, \
        LSP_PLUGINS_CLIPPER_VERSION_MINOR, \
        LSP_PLUGINS_CLIPPER_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        static const port_item_t clipper_xover_modes[] =
        {
            { "Classic",            "multiband.classic"             },
            { "Linear Phase",       "multiband.linear_phase"        },
            { NULL, NULL }
        };

        static const port_item_t clipper_xover_slopes[] =
        {
            { "LR4 (24 dB/oct)",    "filter.lr_mode.24dbo"          },
            { "LR8 (48 dB/oct)",    "filter.lr_mode.48dbo"          },
            { "LR12 (72 dB/oct)",   "filter.lr_mode.72dbo"          },
            { "LR16 (96 dB/oct)",   "filter.lr_mode.96dbo"          },
            { NULL, NULL }
        };

        static const port_item_t clipper_prefilter_slopes[] =
        {
            { "Off",                "filter.lr_mode.off"            },
            { "LR4 (24 dB/oct)",    "filter.lr_mode.24dbo"          },
            { "LR8 (48 dB/oct)",    "filter.lr_mode.48dbo"          },
            { "LR12 (72 dB/oct)",   "filter.lr_mode.72dbo"          },
            { "LR16 (96 dB/oct)",   "filter.lr_mode.96dbo"          },
            { NULL, NULL }
        };

        static const port_item_t clipper_band_selectors[] =
        {
            { "Band 1",             "clipper.band.1"                },
            { "Band 2",             "clipper.band.2"                },
            { "Band 3",             "clipper.band.3"                },
            { "Band 4",             "clipper.band.4"                },
            { NULL, NULL }
        };

    #define CLIPPER_COMMON \
        BYPASS, \
        IN_GAIN, \
        OUT_GAIN, \
        CONTROL("thresh", "Clipping threshold", U_DB, clipper::THRESHOLD), \
        SWITCH("boost", "Boosting mode", 1.0f), \
        COMBO("mode", "Crossover operating mode", 1, clipper_xover_modes), \
        COMBO("slope", "Crossover filter slope", 1, clipper_xover_slopes), \
        LOG_CONTROL("react", "FFT reactivity", U_MSEC, clipper::REACT_TIME), \
        AMP_GAIN("shift", "Shift gain", 1.0f, 100.0f), \
        LOG_CONTROL("zoom", "Graph zoom", U_GAIN_AMP, clipper::ZOOM), \
        COMBO("hpf_m", "High-pass pre-filter mode", 0, clipper_prefilter_slopes), \
        LOG_CONTROL("hpf_f", "High-pass pre-filter frequency", U_HZ, clipper::HPF_FREQ), \
        LOG_CONTROL("sf1", "Split frequency 1", U_HZ, clipper::SPLIT1), \
        LOG_CONTROL("sf2", "Split frequency 2", U_HZ, clipper::SPLIT2), \
        LOG_CONTROL("sf3", "Split frequency 3", U_HZ, clipper::SPLIT3), \
        COMBO("lpf_m", "High-pass pre-filter mode", 0, clipper_prefilter_slopes), \
        LOG_CONTROL("lpf_f", "Low-pass pre-filter frequency", U_HZ, clipper::LPF_FREQ), \
        SWITCH("ebe", "Enable extra band", 0), \
        COMBO("bsel", "Band selector", 0, clipper_band_selectors), \
        SWITCH("flt", "Band filter curves", 1.0f)

    #define CLIPPER_BAND(id, label) \
        SWITCH("bs" id, "Solo band" label, 0.0f), \
        SWITCH("bm" id, "Mute band" label, 0.0f), \
        MESH("bfc" id, "Band frequency chart" label, 2, clipper::FFT_MESH_POINTS + 2)

    #define CLIPPER_STEREO_COMMON \
        CONTROL("slink", "Stereo link", U_PERCENT, clipper::STEREO_LINK)

    #define CLIPPER_ANALYSIS(id, label) \
        SWITCH("ife" id, "Input FFT graph enable" label, 1.0f), \
        SWITCH("ofe" id, "Output FFT graph enable" label, 1.0f), \
        MESH("ifg" id, "Input FFT graph" label, 2, clipper::FFT_MESH_POINTS + 2), \
        MESH("ofg" id, "Output FFT graph" label, 2, clipper::FFT_MESH_POINTS)

        //-------------------------------------------------------------------------
        // Plugin metadata

        static const port_t clipper_mono_ports[] =
        {
            PORTS_MONO_PLUGIN,
            CLIPPER_COMMON,

            CLIPPER_BAND("_1", ""),
            CLIPPER_BAND("_2", ""),
            CLIPPER_BAND("_3", ""),
            CLIPPER_BAND("_4", ""),

            CLIPPER_ANALYSIS("", ""),

            PORTS_END
        };

        static const port_t clipper_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,
            CLIPPER_COMMON,
            CLIPPER_STEREO_COMMON,

            CLIPPER_BAND("_1", ""),
            CLIPPER_BAND("_2", ""),
            CLIPPER_BAND("_3", ""),
            CLIPPER_BAND("_4", ""),

            CLIPPER_ANALYSIS("_l", " Left"),
            CLIPPER_ANALYSIS("_r", " Right"),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_DYNAMICS, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_MASTERING, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_MASTERING, CF_STEREO, -1 };

        const meta::bundle_t clipper_bundle =
        {
            "clipper",
            "Clipper",
            B_DYNAMICS,
            "", // TODO: provide ID of the video on YouTube
            "Advanced peak clipping tool to maximize output loudness"
        };

        const plugin_t clipper_mono =
        {
            "Clipper Mono",
            "Clipper Mono",
            "CL1M",
            &developers::v_sadovnikov,
            "clipper_mono",
            LSP_LV2_URI("clipper_mono"),
            LSP_LV2UI_URI("clipper_mono"),
            "cl1m",
            LSP_LADSPA_CLIPPER_BASE + 0,
            LSP_LADSPA_URI("clipper_mono"),
            LSP_CLAP_URI("clipper_mono"),
            LSP_PLUGINS_CLIPPER_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE,
            clipper_mono_ports,
            "dynamics/clipper.xml",
            NULL,
            mono_plugin_port_groups,
            &clipper_bundle
        };

        const plugin_t clipper_stereo =
        {
            "Pluginschablone Stereo",
            "Plugin Template Stereo",
            "PS1S",
            &developers::v_sadovnikov,
            "clipper_stereo",
            LSP_LV2_URI("clipper_stereo"),
            LSP_LV2UI_URI("clipper_stereo"),
            "cl1s",
            LSP_LADSPA_CLIPPER_BASE + 1,
            LSP_LADSPA_URI("clipper_stereo"),
            LSP_CLAP_URI("clipper_stereo"),
            LSP_PLUGINS_CLIPPER_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE,
            clipper_stereo_ports,
            "dynamics/clipper.xml",
            NULL,
            stereo_plugin_port_groups,
            &clipper_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



