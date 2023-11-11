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
        //-------------------------------------------------------------------------
        // Plugin metadata

        // NOTE: Port identifiers should not be longer than 7 characters as it will overflow VST2 parameter name buffers
        static const port_t clipper_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            // Input controls
            BYPASS,
            IN_GAIN,
            OUT_GAIN,

            PORTS_END
        };

        // NOTE: Port identifiers should not be longer than 7 characters as it will overflow VST2 parameter name buffers
        static const port_t clipper_stereo_ports[] =
        {
            // Input and output audio ports
            PORTS_STEREO_PLUGIN,

            // Input controls
            BYPASS,
            IN_GAIN,
            OUT_GAIN,

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



