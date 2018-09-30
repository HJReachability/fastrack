# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from __future__ import with_statement
from __future__ import print_function

import os
import sys
import time

from . import rdcore


def _href(location, text):
    return '<a href="%(location)s">%(text)s</a>' % locals()


def link_name(rd_config):
    """
    returns the name to display as for the generated kind of API.
    :param rdconfig: dict as parsed from manifest.xml
    :returns: label, ``str``
    """
    if 'name' in rd_config:
        n = rd_config['name']
    else:
        if rd_config['builder'] == 'doxygen':
            return 'C++ API'
        elif rd_config['builder'] in ['epydoc', 'sphinx']:
            return 'Python API'
        elif rd_config['builder'] in ['external']:
            return rd_config.get('external_label', 'External')
        else:
            return rd_config['builder']
    return n


def output_location(config):
    """
    :returns: uri, ``str``
    """
    if config['builder'] == 'external':
        return config.get('external_url', None)
    else:
        return config.get('output_dir', None)


def generate_links(package, manifest, rd_configs):
    """
    :param package: str packagename
    :param manifest: manifest object
    :param rd_configs: [dict] package manifest rosdoc configs
    :returns: [str] list of html snippets
    """
    config_list = [c for c in rd_configs.itervalues() if c['builder'] != 'rosmake']
    output_dirs = [output_location(c) for c in config_list]
    # filter out empties
    output_dirs = [d for d in output_dirs if d and d != '.']

    # length check. if these are unequal, cannot generate landing
    # page. this is often true if the config is merely generating
    # local.
    if len(output_dirs) != len(config_list):
        return None

    links = [_href(d, link_name(c)) for c, d in zip(config_list, output_dirs)]

    url = manifest.url
    if url:
        links.append(_href(url, '%s Package Documentation' % package))
    return links


def generate_landing_page(package, manifest, rd_configs, output_dir):
    """
    Generate landing page in the event that there are multiple documentation sets
    :returns: list of packages for which there are landing pages generated
    """
    template = rdcore.load_tmpl('landing.template')
    #print("landing_page: packages are", ctx.packages.keys())
    try:
        links = generate_links(package, manifest, rd_configs)
        # if links is empty, it means that the rd_configs builds
        # to the base directory and no landing page is required
        # (or it means that the config is corrupt)
        if not links:
            #print("ignoring landing page for", package)
            return

        html_dir = output_dir
        #print("generating landing page", html_dir)

        if not os.path.isdir(html_dir):
            os.makedirs(html_dir)

        links_html = '\n'.join(['<li class="landing-li">%s</li>' % l for l in links])
        date = str(time.strftime('%a, %d %b %Y %H:%M:%S'))
        tempvars = {
            '$package': package,
            '$links': links_html,
            '$date': date,
                }

        with open(os.path.join(html_dir, 'index.html'), 'w') as f:
            f.write(rdcore.instantiate_template(template, tempvars))

    except Exception, e:
        print("Unable to generate landing_page for [%s]:\n\t%s" % (package, str(e)), file=sys.stderr)
        raise
