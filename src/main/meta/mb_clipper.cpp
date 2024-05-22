/*
 * Copyright (C) 2024 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2024 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/mb_clipper.h>

#define LSP_PLUGINS_MB_CLIPPER_VERSION_MAJOR       1
#define LSP_PLUGINS_MB_CLIPPER_VERSION_MINOR       0
#define LSP_PLUGINS_MB_CLIPPER_VERSION_MICRO       4

#define LSP_PLUGINS_MB_CLIPPER_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_MB_CLIPPER_VERSION_MAJOR, \
        LSP_PLUGINS_MB_CLIPPER_VERSION_MINOR, \
        LSP_PLUGINS_MB_CLIPPER_VERSION_MICRO  \
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

        static const port_item_t clipper_tab_selectors[] =
        {
            { "Band 1",             "mb_clipper.band.1"                     },
            { "Band 2",             "mb_clipper.band.2"                     },
            { "Band 3",             "mb_clipper.band.3"                     },
            { "Band 4",             "mb_clipper.band.4"                     },
            { "Output Clipper",     "mb_clipper.output_clipper"             },
            { NULL, NULL }
        };

        static const port_item_t sigmoid_functions[] =
        {
            { "Hard clip",          "sigmoid.hardclip"                      },
            { "Parabolic",          "sigmoid.parabolic"                     },
            { "Sine",               "sigmoid.sine"                          },
            { "Logistic",           "sigmoid.logistic"                      },
            { "Arctangent",         "sigmoid.arctangent"                    },
            { "Hyperbolic tangent", "sigmoid.hyperbolic_tangent"            },
            { "Guidermannian",      "sigmoid.guidermannian"                 },
            { "Error function",     "sigmoid.error_function"                },
            { "Smoothstep",         "sigmoid.smoothstep"                    },
            { "Smootherstep",       "sigmoid.smootherstep"                  },
            { "Circle",             "sigmoid.circle"                        },

            { NULL, NULL }
        };

        static port_item_t clipper_dither_modes[] =
        {
            { "None",               "dither.none"                           },
            { "7bit",               "dither.bits.7"                         },
            { "8bit",               "dither.bits.8"                         },
            { "11bit",              "dither.bits.11"                        },
            { "12bit",              "dither.bits.12"                        },
            { "15bit",              "dither.bits.15"                        },
            { "16bit",              "dither.bits.16"                        },
            { "23bit",              "dither.bits.23"                        },
            { "24bit",              "dither.bits.24"                        },
            { NULL, NULL }
        };

    #define CLIPPER_COMMON \
        BYPASS, \
        IN_GAIN, \
        OUT_GAIN, \
        SWITCH("lufs_on", "Enable input LUFS limitation", 1.0f), \
        CONTROL("lufs_th", "Input LUFS limiter threshold", U_LUFS, mb_clipper::LUFS_THRESH), \
        LUFS_METER("lufs_il", "Input LUFS value", 24.0f), \
        METER_OUT_GAIN("lufs_gr", "Input LUFS gain reduction", GAIN_AMP_0_DB), \
        LUFS_METER("lufs_ol", "Output LUFS value", 24.0f), \
        CONTROL("thresh", "Clipping threshold", U_DB, mb_clipper::THRESHOLD), \
        SWITCH("boost", "Boosting mode", 1.0f), \
        COMBO("mode", "Crossover operating mode", 1, clipper_xover_modes), \
        COMBO("slope", "Crossover filter slope", 1, clipper_xover_slopes), \
        LOG_CONTROL("react", "FFT reactivity", U_MSEC, mb_clipper::REACT_TIME), \
        AMP_GAIN("shift", "Shift gain", 1.0f, 100.0f), \
        LOG_CONTROL("zoom", "Graph zoom", U_GAIN_AMP, mb_clipper::ZOOM), \
        COMBO("hpf_m", "High-pass pre-filter mode", 0, clipper_prefilter_slopes), \
        LOG_CONTROL("hpf_f", "High-pass pre-filter frequency", U_HZ, mb_clipper::HPF_FREQ), \
        LOG_CONTROL("xf_1", "Split frequency 1", U_HZ, mb_clipper::SPLIT1), \
        LOG_CONTROL("ol_1", "Overdrive protection link 1", U_GAIN_AMP, mb_clipper::ODP_LINK), \
        LOG_CONTROL("xf_2", "Split frequency 2", U_HZ, mb_clipper::SPLIT2), \
        LOG_CONTROL("ol_2", "Overdrive protection link 2", U_GAIN_AMP, mb_clipper::ODP_LINK), \
        LOG_CONTROL("xf_3", "Split frequency 3", U_HZ, mb_clipper::SPLIT3), \
        LOG_CONTROL("ol_3", "Overdrive protection link 3", U_GAIN_AMP, mb_clipper::ODP_LINK), \
        COMBO("lpf_m", "Low-pass pre-filter mode", 0, clipper_prefilter_slopes), \
        LOG_CONTROL("lpf_f", "Low-pass pre-filter frequency", U_HZ, mb_clipper::LPF_FREQ), \
        SWITCH("ebe", "Enable extra band", 0), \
        SWITCH("oclip", "Enable output clipper", 1), \
        COMBO("tsel", "Tab selector", 4, clipper_tab_selectors), \
        SWITCH("flt", "Band filter curves", 1.0f), \
        COMBO("dither", "Dithering mode", 0, clipper_dither_modes), \
        SWITCH("clog", "Clipper logarithmic display", 1.0f)

    #define CLIPPER_BAND(id, label, resonance) \
        SWITCH("bs" id, "Solo band" label, 0.0f), \
        SWITCH("bm" id, "Mute band" label, 0.0f), \
        CONTROL("pa" id, "Band preamp gain" label, U_DB, mb_clipper::PREAMP), \
        SWITCH("lo" id, "Enable input LUFS limitation" label, 1.0f), \
        CONTROL("lt" id, "Input LUFS limiter threshold" label, U_LUFS, mb_clipper::LUFS_THRESH), \
        LUFS_METER("li" id, "Input LUFS value" label, 24.0f), \
        METER_OUT_GAIN("lg" id, "Input LUFS gain reduction" label, GAIN_AMP_0_DB), \
        SWITCH("op" id, "Overdrive protection" label, 1.0f), \
        CONTROL("th" id, "Overdrive protection threshold" label, U_DB, mb_clipper::ODP_THRESHOLD), \
        CONTROL("kn" id, "Overdrive protection knee" label, U_DB, mb_clipper::ODP_KNEE), \
        LOG_CONTROL("rs" id, "Overdrive protection resonance" label, U_HZ, mb_clipper::resonance), \
        MESH("opc" id, "Overdrive protection chart" label, 2, mb_clipper::CURVE_MESH_POINTS), \
        SWITCH("ce" id, "Clipper enable" label, 1.0f), \
        COMBO("cf" id, "Clipper sigmoid function" label, 2.0f, sigmoid_functions), \
        LOG_CONTROL("ct" id, "Clipper sigmoid threshold" label, U_GAIN_AMP, mb_clipper::CLIP_THRESHOLD), \
        CONTROL("cp" id, "Clipper sigmoid pumping" label, U_DB, mb_clipper::CLIP_PUMPING), \
        MESH("cfc" id, "Clipper sigmoid function chart" label, 4, mb_clipper::CURVE_MESH_POINTS), \
        MESH("bfc" id, "Band frequency chart" label, 2, mb_clipper::FFT_MESH_POINTS + 2), \
        CONTROL("mk" id, "Band makeup gain" label, U_DB, mb_clipper::MAKEUP)

    #define OUTPUT_CLIPPER \
        SWITCH("lo", "Enable output clipper LUFS limitation", 1.0f), \
        CONTROL("lt", "Output clipper LUFS limiter threshold", U_LUFS, mb_clipper::LUFS_THRESH), \
        LUFS_METER("li", "Output clipper LUFS value", 24.0f), \
        METER_OUT_GAIN("lg", "Output clipper LUFS gain reduction", GAIN_AMP_0_DB), \
        SWITCH("op", "Output overdrive protection", 1.0f), \
        CONTROL("th", "Output overdrive protection threshold", U_DB, mb_clipper::ODP_THRESHOLD), \
        CONTROL("kn", "Output overdrive protection knee", U_DB, mb_clipper::ODP_KNEE), \
        LOG_CONTROL("or", "Output overdrive protection reactivity", U_MSEC, mb_clipper::ODP_REACT), \
        MESH("opc", "Output overdrive protection chart", 2, mb_clipper::CURVE_MESH_POINTS), \
        SWITCH("ce", "Output clipper enable", 1.0f), \
        COMBO("cf", "Output clipper sigmoid function", 2.0f, sigmoid_functions), \
        LOG_CONTROL("ct", "Output clipper sigmoid threshold", U_GAIN_AMP, mb_clipper::CLIP_THRESHOLD), \
        CONTROL("cp", "Output clipper sigmoid pumping", U_DB, mb_clipper::CLIP_PUMPING), \
        MESH("cfc", "Output clipper sigmoid function chart", 4, mb_clipper::CURVE_MESH_POINTS)

    #define CLIPPER_METERS(id, label) \
        METER_OUT_GAIN("ilm" id, "Input level meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("olm" id, "Output level meter" label, GAIN_AMP_P_36_DB), \
        METER_GAIN_DFL("grm" id, "Gain reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
        METER_OUT_GAIN("odx" id, "Overdrive protection input meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("ody" id, "Overdrive protection output meter" label, GAIN_AMP_P_36_DB), \
        METER_GAIN_DFL("odr" id, "Overdrive protection reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
        METER_OUT_GAIN("cfx" id, "Clipping function input meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("cfy" id, "Clipping function output meter" label, GAIN_AMP_P_36_DB), \
        METER_GAIN_DFL("cfr" id, "Clipping function reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
        MESH("ctg" id, "Clipper time graph" label, 4, mb_clipper::TIME_MESH_POINTS + 4)

    #define CLIPPER_STEREO_BAND(id, label, resonance, link) \
        CONTROL_DFL("bl" id, "Band stereo link" label, U_PERCENT, mb_clipper::STEREO_LINK, link), \
        CLIPPER_BAND(id, label, resonance)

    #define OUTPUT_STEREO_CLIPPER \
        CONTROL_DFL("slink", "Stereo link", U_PERCENT, mb_clipper::STEREO_LINK, 50.0f), \
        OUTPUT_CLIPPER

    #define OSCILLOSCOPE_SWITCHES(id, label) \
        SWITCH("ilg" id, "Input level graph enable" label, 1.0f), \
        SWITCH("olg" id, "Output level graph enable" label, 1.0f), \
        SWITCH("grg" id, "Gain reduction graph enable" label, 1.0f)

    #define CLIPPER_ANALYSIS(id, label) \
        METER_OUT_GAIN("ism" id, "Input signal meter" label, GAIN_AMP_P_36_DB), \
        METER_OUT_GAIN("osm" id, "Output signal meter" label, GAIN_AMP_P_36_DB), \
        SWITCH("ife" id, "Input FFT graph enable" label, 1.0f), \
        SWITCH("ofe" id, "Output FFT graph enable" label, 1.0f), \
        MESH("ifg" id, "Input FFT graph" label, 2, mb_clipper::FFT_MESH_POINTS + 2), \
        MESH("ofg" id, "Output FFT graph" label, 2, mb_clipper::FFT_MESH_POINTS), \
        MESH("grc" id, "Crossover gain reduction chart" label, 2, mb_clipper::FFT_MESH_POINTS)

        //-------------------------------------------------------------------------
        // Plugin metadata

        static const port_t clipper_mono_ports[] =
        {
            PORTS_MONO_PLUGIN,
            CLIPPER_COMMON,

            CLIPPER_BAND("_1", " Band 1", ODP_REACT1),
            CLIPPER_BAND("_2", " Band 2", ODP_REACT2),
            CLIPPER_BAND("_3", " Band 3", ODP_REACT3),
            CLIPPER_BAND("_4", " Band 4", ODP_REACT4),
            OUTPUT_CLIPPER,

            OSCILLOSCOPE_SWITCHES("", ""),

            CLIPPER_ANALYSIS("", ""),

            CLIPPER_METERS("_1", " Band 1"),
            CLIPPER_METERS("_2", " Band 2"),
            CLIPPER_METERS("_3", " Band 3"),
            CLIPPER_METERS("_4", " Band 4"),
            CLIPPER_METERS("", " Output"),

            PORTS_END
        };

        static const port_t clipper_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,
            CLIPPER_COMMON,

            CLIPPER_STEREO_BAND("_1", " Band 1", ODP_REACT1, 100.0f),
            CLIPPER_STEREO_BAND("_2", " Band 2", ODP_REACT2, 50.0f),
            CLIPPER_STEREO_BAND("_3", " Band 3", ODP_REACT3, 25.0f),
            CLIPPER_STEREO_BAND("_4", " Band 4", ODP_REACT4, 0.0f),
            OUTPUT_STEREO_CLIPPER,

            OSCILLOSCOPE_SWITCHES("_l", " Left"),
            OSCILLOSCOPE_SWITCHES("_r", " Right"),

            CLIPPER_ANALYSIS("_l", " Left"),
            CLIPPER_ANALYSIS("_r", " Right"),

            CLIPPER_METERS("_1l", " Band 1 Left"),
            CLIPPER_METERS("_2l", " Band 2 Left"),
            CLIPPER_METERS("_3l", " Band 3 Left"),
            CLIPPER_METERS("_4l", " Band 4 Left"),
            CLIPPER_METERS("_1r", " Right"),
            CLIPPER_METERS("_2r", " Right"),
            CLIPPER_METERS("_3r", " Right"),
            CLIPPER_METERS("_4r", " Right"),
            CLIPPER_METERS("_l", " Output Left"),
            CLIPPER_METERS("_r", " Output Right"),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_DYNAMICS, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_MASTERING, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_MASTERING, CF_STEREO, -1 };

        const meta::bundle_t mb_clipper_bundle =
        {
            "mb_clipper",
            "Multiband Clipper",
            B_MB_DYNAMICS,
            "jbnGSgoVIkg",
            "Advanced multiband peak clipping tool for maximization of output loudness"
        };

        const plugin_t mb_clipper_mono =
        {
            "Multiband Clipper Mono",
            "Multiband Clipper Mono",
            "MB Clipper Mono",
            "MBCL1M",
            &developers::v_sadovnikov,
            "mb_clipper_mono",
            LSP_LV2_URI("mb_clipper_mono"),
            LSP_LV2UI_URI("mb_clipper_mono"),
            "CL1M",
            LSP_VST3_UID("mbcl1m  CL1M"),
            LSP_VST3UI_UID("mbcl1m  CL1M"),
            LSP_LADSPA_MB_CLIPPER_BASE + 0,
            LSP_LADSPA_URI("mb_clipper_mono"),
            LSP_CLAP_URI("mb_clipper_mono"),
            LSP_PLUGINS_MB_CLIPPER_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            clipper_mono_ports,
            "dynamics/clipper/multiband/mono.xml",
            NULL,
            mono_plugin_port_groups,
            &mb_clipper_bundle
        };

        const plugin_t mb_clipper_stereo =
        {
            "Multiband Clipper Stereo",
            "Multiband Clipper Stereo",
            "MB Clipper Stereo",
            "MBCL1S",
            &developers::v_sadovnikov,
            "mb_clipper_stereo",
            LSP_LV2_URI("mb_clipper_stereo"),
            LSP_LV2UI_URI("mb_clipper_stereo"),
            "CL1S",
            LSP_VST3_UID("mbcl1s  CL1S"),
            LSP_VST3UI_UID("mbcl1s  CL1S"),
            LSP_LADSPA_MB_CLIPPER_BASE + 1,
            LSP_LADSPA_URI("mb_clipper_stereo"),
            LSP_CLAP_URI("mb_clipper_stereo"),
            LSP_PLUGINS_MB_CLIPPER_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            clipper_stereo_ports,
            "dynamics/clipper/multiband/stereo.xml",
            NULL,
            stereo_plugin_port_groups,
            &mb_clipper_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



