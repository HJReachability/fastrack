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

from __future__ import print_function

import sys
import os
import traceback
import yaml
from subprocess import Popen, PIPE
import shutil

NAME = 'rosdoc_lite'

from . import rdcore
from . import epyenator
from . import sphinxenator
from . import landing_page
from . import doxygenator

import rospkg

from catkin_pkg.package import parse_package


def get_optparse(name):
    """
    Retrieve default option parser for rosdoc. Useful if building an
    extended rosdoc tool with additional options.
    """
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] [package_path]", prog=name)
    parser.add_option("-q", "--quiet", action="store_true", default=False,
                      dest="quiet",
                      help="Suppress doxygen errors.")
    parser.add_option("-o", metavar="OUTPUT_DIRECTORY",
                      dest="docdir", default='doc',
                      help="The directory to write documentation to.")
    parser.add_option("-t", "--tagfile", metavar="TAGFILE", dest="tagfile", default=None,
                      help="Path to tag configuration file for Doxygen cross referencing support. Ex: /home/user/tagfiles_list.yaml")
    parser.add_option("-g", "--generate_tagfile", default=None, dest="generate_tagfile",
                      help="If specified, will generate a doxygen tagfile in this location. Ex: /home/user/tags/package.tag")
    parser.add_option("-p", "--generate_qhp", action="store_true", default=False, dest="generate_qhp",
                      help="If specified, will generate the Qt Help Project file index.qhp")
    return parser


def load_rd_config(path, manifest):
    #load in any external config files
    rd_config = {}
    exported_configs = manifest.get_export('rosdoc', 'config')
    if exported_configs:
        #This just takes the last listed config export
        for exported_config in manifest.get_export('rosdoc', 'config'):
            try:
                exported_config = exported_config.replace('${prefix}', path)
                config_path = os.path.join(path, exported_config)
                with open(config_path, 'r') as config_file:
                    rd_config = yaml.load(config_file)
            except Exception as e:
                sys.stderr.write("ERROR: unable to load rosdoc config file [%s]: %s\n" % (config_path, str(e)))
    #we'll check if a 'rosdoc.yaml' file exists in the directory
    elif os.path.isfile(os.path.join(path, 'rosdoc.yaml')):
        with open(os.path.join(path, 'rosdoc.yaml'), 'r') as config_file:
            rd_config = yaml.load(config_file)

    return rd_config


def generate_build_params(rd_config, package):
    build_params = {}
    #if there's no config, we'll just build doxygen with the defaults
    if not rd_config:
        build_params['doxygen'] = {'builder': 'doxygen', 'output_dir': '.'}
    #make sure that we have a valid rd_config
    elif type(rd_config) != list:
        sys.stderr.write("WARNING: package [%s] had an invalid rosdoc config\n" % (package))
        build_params['doxygen'] = {'builder': 'doxygen', 'output_dir': '.'}
    #generate build parameters for the different types of builders
    else:
        try:
            for target in rd_config:
                build_params[target['builder']] = target
        except KeyError:
            sys.stderr.write("config file for [%s] is invalid, missing required 'builder' key\n" % (package))
        except:
            sys.stderr.write("config file for [%s] is invalid\n" % (package))
            raise

    return build_params


def build_manifest_yaml(manifest, msgs, srvs, actions, output_dir):
    # by default, assume that packages are on wiki
    m_yaml = {}
    m_yaml['authors'] = manifest.author or ''
    m_yaml['maintainers'] = manifest.maintainer or ''
    m_yaml['brief'] = manifest.brief or ''
    m_yaml['depends'] = manifest.depends or ''
    m_yaml['description'] = manifest.description or ''
    m_yaml['license'] = manifest.license or ''
    m_yaml['msgs'] = msgs
    m_yaml['srvs'] = srvs
    m_yaml['actions'] = actions
    m_yaml['url'] = manifest.url or ''
    m_yaml['bugtracker'] = manifest.bugtracker or ''
    m_yaml['repo_url'] = manifest.repo_url or ''
    external_docs = manifest.get_export('doxymaker', 'external')
    if external_docs:
        m_yaml['external_docmentation'] = external_docs

    metapackage = [e for e in manifest.exports if e.tagname == 'metapackage']
    if metapackage:
        m_yaml['package_type'] = 'metapackage'
    else:
        m_yaml['package_type'] = 'package'

    deprecated = [e for e in manifest.exports if e.tagname == 'deprecated']
    if deprecated:
        m_yaml['deprecated'] = deprecated[0].content or "This package is deprecated."

    with open(os.path.join(output_dir, 'manifest.yaml'), 'w') as f:
        yaml.safe_dump(m_yaml, f, default_flow_style=False)


def generate_docs(path, package, manifest, output_dir, tagfile, generate_tagfile, generate_qhp=False, quiet=True):
    """
    Generates API docs by invoking plugins with context

    :returns: a list of filenames/paths that is the union set of all results of plugin invocations
    """
    plugins = [
        ('doxygen', doxygenator.generate_doxygen),
        ('epydoc', epyenator.generate_epydoc),
        ('sphinx', sphinxenator.generate_sphinx)
               ]

    #load any rosdoc configuration files
    rd_config = load_rd_config(path, manifest)

    #put the rd_config into a form that's easier to use with plugins
    build_params = generate_build_params(rd_config, package)

    #throw in tagfiles for doxygen if it is going to be built
    if 'doxygen' in build_params:
        if tagfile:
            build_params['doxygen']['tagfile_spec'] = tagfile

        if generate_tagfile:
            build_params['doxygen']['generate_tagfile'] = generate_tagfile

        if generate_qhp:
            build_params['doxygen']['generate_qhp'] = generate_qhp

    print(build_params)

    html_dir = os.path.join(output_dir, 'html')

    for plugin_name, plugin in plugins:
        #check to see if we're supposed to build each plugin
        if plugin_name in build_params:
            try:
                plugin(path, package, manifest, build_params[plugin_name], html_dir, quiet)
            except Exception, e:
                traceback.print_exc()
                print("plugin [%s] failed" % (plugin_name), file=sys.stderr)

    #Generate a landing page for the package, requires passing all the build_parameters on
    landing_page.generate_landing_page(package, manifest, build_params, html_dir)

    #Generate documentation for messages and store the messages successfully generated
    from . import msgenator
    msgs, srvs, actions = msgenator.generate_msg_docs(package, path, manifest, html_dir)

    #Write meta data for the package to a yaml file for use by external tools
    build_manifest_yaml(manifest, msgs, srvs, actions, output_dir)

    #We'll also write the message stylesheet that the landing page and message docs use
    styles_name = 'msg-styles.css'
    styles_in = os.path.join(rdcore.get_templates_dir(), styles_name)
    styles_css = os.path.join(html_dir, styles_name)
    print("copying %s to %s" % (styles_in, styles_css))
    shutil.copyfile(styles_in, styles_css)


def is_catkin(path):
    return os.path.isfile(os.path.join(path, 'package.xml'))


def main():
    parser = get_optparse(NAME)
    options, args = parser.parse_args()

    if len(args) != 1:
        print("Please give %s exactly one package path" % NAME)
        parser.print_help()
        sys.exit(1)

    path = os.path.realpath(args[0])
    package = os.path.basename(path)
    pkg_desc = get_pkg_desc(path)

    manifest = rdcore.PackageInformation(pkg_desc)
    print("Documenting %s located here: %s" % (package, path))

    try:
        generate_docs(path, package, manifest, options.docdir, options.tagfile, options.generate_tagfile, options.generate_qhp, options.quiet)
        print("Done documenting %s you can find your documentation here: %s" % (package, os.path.realpath(options.docdir)))
    except:
        traceback.print_exc()
        sys.exit(1)


def get_pkg_desc(path):
    #Check whether we've got a catkin or non-catkin package
    if is_catkin(path):
        pkg_desc = parse_package(path)
        print("Documenting a catkin package")
    else:
        rp = rospkg.RosPack()
        package = os.path.basename(path)
        try:
            ros_path = os.path.realpath(rp.get_path(package))
        except rospkg.common.ResourceNotFound as e:
            sys.stderr.write("Rospack could not find the %s. Are you sure it's on your ROS_PACKAGE_PATH?\n" % package)
            sys.exit(1)
        if ros_path != path:
            sys.stderr.write("The path passed in does not match that returned \
                             by rospack. Requested path: %s. Rospack path: %s.\n" % (path, ros_path))
            sys.exit(1)
        pkg_desc = rp.get_manifest(package)
        print("Documenting a non-catkin package")
    return pkg_desc


def get_generator_output_folders(path):
    pkg_desc = get_pkg_desc(path)
    manifest = rdcore.PackageInformation(pkg_desc)

    # load any rosdoc configuration files
    rd_config = load_rd_config(path, manifest)

    # put the rd_config into a form that's easier to use with plugins
    package = os.path.basename(path)
    build_params = generate_build_params(rd_config, package)
    print('build_params', build_params)

    data = {}
    for builder, params in build_params.items():
        if 'output_dir' in params:
            output_dir = params['output_dir']
            if output_dir != '.':
                data[builder] = output_dir
    return data
