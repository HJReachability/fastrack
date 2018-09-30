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

import os
import sys


from catkin_pkg.package import Package, Export
import rospkg

#Helper object to pull the information rosdoc needs from both manifest and
#package objects


def convert_manifest_export(man_export):
    e = Export(man_export.tag, man_export.str)
    e.attributes = man_export.attrs
    return e


class PackageInformation(object):
    __slots__ = ['license', 'maintainer', 'author', 'description', 'status', 'brief', 'url', 'is_catkin', 'exports', 'depends', 'bugtracker', 'repo_url']

    def __init__(self, pkg_desc):
        if isinstance(pkg_desc, Package):
            self.create_from_package(pkg_desc)
        elif isinstance(pkg_desc, rospkg.Manifest):
            self.create_from_manifest(pkg_desc)

    def create_from_package(self, package):
        self.license = ', '.join(package.licenses)
        self.author =  ', '.join([('%s <%s>' % (a.name, a.email) if a.email is not None else a.name) for a in package.authors])
        self.maintainer =  ', '.join([('%s <%s>' % (m.name, m.email) if m.email is not None else m.name) for m in package.maintainers])
        self.description = package.description

        #we'll just use the first url with type website or the first URL of any type
        websites = [url.url for url in package.urls if url.type == 'website']
        self.url = websites[0] if websites else None

        #we'll also check if there's a bugtracker URL
        bugtracker = [url.url for url in package.urls if url.type == 'bugtracker']
        self.bugtracker = bugtracker[0] if bugtracker else None

        #we'll also check if there's a bugtracker URL
        repo_url = [url.url for url in package.urls if url.type == 'repository']
        self.repo_url = repo_url[0] if repo_url else None

        self.is_catkin = True
        self.exports = package.exports
        self.status = None
        self.brief = None
        self.depends = []
        self.depends.extend([dep.name for dep in package.build_depends])
        self.depends.extend([dep.name for dep in package.buildtool_depends])
        self.depends.extend([dep.name for dep in package.run_depends])
        self.depends.extend([dep.name for dep in package.test_depends])
        self.depends = list(set(self.depends))

    def create_from_manifest(self, manifest):
        self.repo_url = None
        self.bugtracker = None
        self.license = manifest.license
        self.maintainer = None
        self.author = manifest.author
        self.description = manifest.description
        self.status = manifest.status
        self.brief = manifest.brief
        self.url = manifest.url
        self.is_catkin = manifest.is_catkin
        self.exports = [convert_manifest_export(e) for e in manifest.exports]
        self.depends = [dep.name for dep in manifest.depends]
        self.depends.extend([dep.name for dep in manifest.rosdeps])
        self.depends = list(set(self.depends))

    def get_export(self, tag, attr):
        vals = [e.attributes[attr] for e in self.exports
                if e.tagname == tag if attr in e.attributes]
        return vals


def html_path(package, docdir):
    return os.path.join(docdir, package, 'html')


################################################################################
# TEMPLATE ROUTINES
import pkg_resources
import kitchen

_TEMPLATES_DIR = 'templates'
_PACKAGE_NAME = 'rosdoc_lite'


def get_templates_dir():
    return os.path.join(os.path.dirname(__file__), _TEMPLATES_DIR)


def load_tmpl(filename):
    """
    looks up file within rosdoc ROS package,
    return its content, may sys.exit on error.
    Contents are cached with filename as key.

    :returns: cached file contents
    """
    filename = os.path.join(get_templates_dir(), filename)
    if not os.path.isfile(filename):
        sys.stderr.write("Cannot locate template file '%s'\n" % (filename))
        sys.exit(1)
    with open(filename, 'r') as f:
        content = f.read()
        if not content:
            sys.stderr.write("Template file '%s' is empty\n" % (filename))
            sys.exit(1)
        return content


def instantiate_template(tmpl, tempvars):
    """
    looks up file within rosdoc_lite package, return its content, may sys.exit on error
    """
    for k, v in tempvars.iteritems():
        tmpl = tmpl.replace(k, kitchen.text.converters.to_unicode(v))
    return tmpl
