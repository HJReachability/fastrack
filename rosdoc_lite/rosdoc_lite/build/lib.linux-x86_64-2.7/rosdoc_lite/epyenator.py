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
import subprocess
import rospkg
from . import python_paths


def generate_epydoc(path, package, manifest, rd_config, output_dir, quiet):
    """
    Main entrypoint into creating Epydoc documentation
    """
    #make sure that we create docs in a subdirectory if requested
    html_dir = os.path.join(output_dir, rd_config.get('output_dir', '.'))

    try:
        if not os.path.isdir(html_dir):
            os.makedirs(html_dir)

        command = ['epydoc', '--html', package, '-o', html_dir]
        if 'exclude' in rd_config:
            for s in rd_config['exclude']:
                command.extend(['--exclude', s])

        if 'config' in rd_config:
            command.extend(['--config', os.path.join(path, rd_config['config']) ])
        else:
            # default options
            command.extend(['--inheritance', 'included', '--no-private'])

        # determine the python path of the package
        paths = python_paths.generate_python_path(package, rospkg.RosPack(), manifest)
        if not quiet:
            print('generate_epydoc() paths: %s' % ', '.join(paths))
        env = os.environ.copy()
        additional_packages = [p for p in paths if os.path.exists(p)]
        if not quiet:
            print('generate_epydoc() additional_packages: %s' % ', '.join(additional_packages))
        if additional_packages:
            env['PYTHONPATH'] = "%s:%s" % (os.pathsep.join(additional_packages), env['PYTHONPATH'])

        if not quiet:
            print('generate_epydoc() PYTHONPATH: %s' % env.get('PYTHONPATH', ''))
            print("epydoc-building %s [%s]" % (package, ' '.join(command)))
        output = subprocess.check_output(command, stderr=subprocess.STDOUT, env=env, cwd=path)
        print(output)
    except subprocess.CalledProcessError as e:
        print(e.output, file=sys.stderr)
        print("Unable to generate epydoc for [%s]. The return code is %d" % (package, e.returncode), file=sys.stderr)
        raise
    except Exception, e:
        print("Unable to generate epydoc for [%s]. This is probably because epydoc is not installed.\nThe exact error is:\n\t%s" % (package, str(e)), file=sys.stderr)
        raise
