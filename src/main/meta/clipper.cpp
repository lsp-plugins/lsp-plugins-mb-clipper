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
            { "Classic",            "multiband.classic"                     },
            { "Linear Phase",       "multiband.linear_phase"                },
            { NULL, NULL }
        };

        static const port_item_t clipper_xover_slopes[] =
        {
            { "LR4 (24 dB/oct)",    "filter.lr_mode.24dbo"                  },
            { "LR8 (48 dB/oct)",    "filter.lr_mode.48dbo"                  },
            { "LR12 (72 dB/oct)",   "filter.lr_mode.72dbo"                  },
            { "LR16 (96 dB/oct)",   "filter.lr_mode.96dbo"                  },
            { NULL, NULL }
        };

        static const port_item_t clipper_prefilter_slopes[] =
        {
            { "Off",                "filter.lr_mode.off"                    },
            { "LR4 (24 dB/oct)",    "filter.lr_mode.24dbo"                  },
            { "LR8 (48 dB/oct)",    "filter.lr_mode.48dbo"                  },
            { "LR12 (72 dB/oct)",   "filter.lr_mode.72dbo"                  },
            { "LR16 (96 dB/oct)",   "filter.lr_mode.96dbo"                  },
            { NULL, NULL }
        };

        static const port_item_t clipper_band_selectors[] =
        {
            { "Band 1",             "clipper.band.1"                        },
            { "Band 2",             "clipper.band.2"                        },
            { "Band 3",             "clipper.band.3"                        },
            { "Band 4",             "clipper.band.4"                        },
            { NULL, NULL }
        };

        static const port_item_t sigmoid_functions[] =
        {
            { "Hard clip",          "clipper.sigmoid.hardclip"              },
            { "Quadratic",          "clipper.sigmoid.quadratic"             },
            { "Sine",               "clipper.sigmoid.sine"                  },
            { "Logistic",           "clipper.sigmoid.logistic"              },
            { "Arctangent",         "clipper.sigmoid.arctangent"            },
            { "Hyperbolic tangent", "clipper.sigmoid.hyperbolic_tangent"    },
            { "Guidermannian",      "clipper.sigmoid.guidermannian"         },
            { "Error function",     "clipper.sigmoid.error_function"        },
            { "Smoothstep",         "clipper.sigmoid.smoothstep"            },
            { "Smootherstep",       "clipper.sigmoid.smootherstep"          },
            { "Circle",             "clipper.sigmoid.circle"                },

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
        LOG_CONTROL("ol1", "Overdrive protection link 1", U_GAIN_AMP, clipper::ODP_LINK), \
        LOG_CONTROL("sf2", "Split frequency 2", U_HZ, clipper::SPLIT2), \
        LOG_CONTROL("ol2", "Overdrive protection link 2", U_GAIN_AMP, clipper::ODP_LINK), \
        LOG_CONTROL("sf3", "Split frequency 3", U_HZ, clipper::SPLIT3), \
        LOG_CONTROL("ol3", "Overdrive protection link 3", U_GAIN_AMP, clipper::ODP_LINK), \
        COMBO("lpf_m", "High-pass pre-filter mode", 0, clipper_prefilter_slopes), \
        LOG_CONTROL("lpf_f", "Low-pass pre-filter frequency", U_HZ, clipper::LPF_FREQ), \
        SWITCH("ebe", "Enable extra band", 0), \
        COMBO("bsel", "Band selector", 0, clipper_band_selectors), \
        SWITCH("flt", "Band filter curves", 1.0f)

    #define CLIPPER_BAND(id, label, resonance) \
        SWITCH("bs" id, "Solo band" label, 0.0f), \
        SWITCH("bm" id, "Mute band" label, 0.0f), \
        SWITCH("op" id, "Overdrive protection" label, 1.0f), \
        CONTROL("th" id, "Overdrive protection threshold" label, U_DB, clipper::ODP_THRESHOLD), \
        CONTROL("kn" id, "Overdrive protection knee" label, U_DB, clipper::ODP_KNEE), \
        CONTROL("mk" id, "Overdrive protection makeup gain" label, U_DB, clipper::ODP_MAKEUP), \
        LOG_CONTROL("rs" id, "Overdrive protection resonance" label, U_HZ, clipper::resonance), \
        MESH("opc" id, "Overdrive protection chart" label, 2, clipper::CURVE_MESH_POINTS), \
        SWITCH("se" id, "Clipper sigmoid function enable" label, 1.0f), \
        SWITCH("sl" id, "Clipper sigmoid function display logarithmic" label, 1.0f), \
        COMBO("sf" id, "Clipper sigmoid function" label, 1.0f, sigmoid_functions), \
        LOG_CONTROL("st" id, "Clipper sigmoid threshold" label, U_GAIN_AMP, clipper::CLIP_THRESHOLD), \
        CONTROL("sp" id, "Clipper sigmoid pumping" label, U_PERCENT, clipper::CLIP_PUMPING), \
        MESH("sfc" id, "Sigmoid function chart" label, 4, clipper::CURVE_MESH_POINTS), \
        MESH("bfc" id, "Band frequency chart" label, 2, clipper::FFT_MESH_POINTS + 2)

    #define CLIPPER_BAND_METERS(id, label) \
        METER_OUT_GAIN("odx" id, "Overdrive protection input meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("ody" id, "Overdrive protection output meter" label, GAIN_AMP_P_36_DB), \
        METER_GAIN_DFL("orm" id, "Overdrive protection reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
        METER_OUT_GAIN("sfx" id, "Sigmoid function input meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("sfy" id, "Sigmoid function output meter" label, GAIN_AMP_P_36_DB), \
        METER_GAIN_DFL("srm" id, "Sigmoid function reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB)

    #define CLIPPER_STEREO_BAND(id, label, resonance, link) \
        CONTROL_DFL("bl" id, "Band stereo link" label, U_PERCENT, clipper::STEREO_LINK, link), \
        CLIPPER_BAND(id, label, resonance)

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

            CLIPPER_BAND("_1", "", ODP_REACT1),
            CLIPPER_BAND("_2", "", ODP_REACT2),
            CLIPPER_BAND("_3", "", ODP_REACT3),
            CLIPPER_BAND("_4", "", ODP_REACT4),

            CLIPPER_ANALYSIS("", ""),

            CLIPPER_BAND_METERS("_1", ""),
            CLIPPER_BAND_METERS("_2", ""),
            CLIPPER_BAND_METERS("_3", ""),
            CLIPPER_BAND_METERS("_4", ""),

            PORTS_END
        };

        static const port_t clipper_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,
            CLIPPER_COMMON,

            CLIPPER_STEREO_BAND("_1", "", ODP_REACT1, 1.0f),
            CLIPPER_STEREO_BAND("_2", "", ODP_REACT2, 0.5f),
            CLIPPER_STEREO_BAND("_3", "", ODP_REACT3, 0.25f),
            CLIPPER_STEREO_BAND("_4", "", ODP_REACT4, 0.0f),

            CLIPPER_ANALYSIS("_l", " Left"),
            CLIPPER_ANALYSIS("_r", " Right"),

            CLIPPER_BAND_METERS("_1l", " Left"),
            CLIPPER_BAND_METERS("_2l", " Left"),
            CLIPPER_BAND_METERS("_3l", " Left"),
            CLIPPER_BAND_METERS("_4l", " Left"),
            CLIPPER_BAND_METERS("_1r", " Right"),
            CLIPPER_BAND_METERS("_2r", " Right"),
            CLIPPER_BAND_METERS("_3r", " Right"),
            CLIPPER_BAND_METERS("_4r", " Right"),

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



