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
from subprocess import Popen, PIPE
import tempfile
import shutil
import yaml

from . import rdcore


def run_doxygen(package, doxygen_file, quiet=False):
    try:
        command = ['doxygen', doxygen_file]
        if quiet:
            Popen(command, stdout=PIPE, stderr=PIPE).communicate()
        else:
            print("doxygen-ating %s [%s]" % (package, ' '.join(command)))
            Popen(command, stdout=PIPE).communicate()
    except OSError:
        #fatal
        print("""\nERROR: It appears that you do not have doxygen installed.
If you are on Ubuntu/Debian, you can install doxygen by typing:

   sudo apt-get install doxygen
""", file=sys.stderr)
        sys.exit(1)


def write_to_file(f, tmpl):
    """utility to write string data to files and handle unicode"""
    try:
        if type(tmpl) == str:
            f.write(tmpl)
        else:  # iso-8859-1 is the declared encoding of doxygen
            f.write(tmpl.encode('utf-8'))
        f.flush()
    except:
        print("ERROR, f[%s], tmpl[%s]" % (f, tmpl))
        raise


def load_manifest_vars(rd_config, package, manifest):
    """readies manifest information for inclusion in doxygen templates"""
    author = license_str = description = status = brief = ''

    # by default, assume that packages are on wiki
    home_url = 'http://wiki.ros.org/%s' % package
    home_url = rd_config.get('homepage', home_url)

    project_link = '<a href="%s">%s</a>' % (home_url, package)
    if manifest is not None:
        license_str = manifest.license or ''
        author = manifest.author or ''
        description = manifest.description or ''
        status = manifest.status or ''

        if manifest.brief:
            brief = ": " + manifest.brief
    else:
        print("no manifest [%s]" % (package))

    return {'$package': package,
            '$projectlink': project_link, '$license': license_str,
            '$description': description, '$brief': brief,
            '$author': author, '$status': status, '$home_url': home_url
            }


def prepare_tagfiles(tagfile_spec, tagfile_dir, output_subfolder):
    """A function that will load a tagfile from either a URL or the filesystem"""
    tagfile_list = []
    with open(tagfile_spec) as f:
        tagfile_list = yaml.load(f)
    tagfile_string = ""
    for tag_pair in tagfile_list:
        print(tag_pair)
        if tag_pair['location'].find("http://") == 0:
            #We need to download the file from somewhere online
            import urllib2
            try:
                ret = urllib2.urlopen(tag_pair['location'])
                if ret.code != 200:
                    print("Could not fetch the tagfile from %s, skipping" % tag_pair['location'], file=sys.stderr)
                    continue
                tagfile_name = os.path.basename(tag_pair['location'])
                tagfile_path = os.path.join(tagfile_dir, tagfile_name)
                tagfile = open(tagfile_path, 'w')
                tagfile.write(ret.read())
                tagfile.close()
                tagfile_string += "%s=%s " % (tagfile_path, get_relative_doc_path(output_subfolder, tag_pair))
            except (urllib2.URLError, urllib2.HTTPError) as e:
                print("Could not fetch the tagfile from %s, skipping" % tag_pair['location'], file=sys.stderr)
                continue
        elif tag_pair['location'].find("file://") == 0:
            tagfile_path = tag_pair['location'][7:]
            tagfile_string += "%s=%s " % (tagfile_path, get_relative_doc_path(output_subfolder, tag_pair))
        else:
            print("Tagfile location only supports http// and file:// prefixes, but you specify %s, skipping" % tag_pair['location'],
                  file=sys.stderr)
    return tagfile_string


def get_relative_doc_path(output_subfolder, tag_pair):
    path = tag_pair['docs_url']
    # prefix the path with as many .. as the output_subfolder is deep
    if output_subfolder != '.':
        output_subfolder_level = len(output_subfolder.split('/'))
        reverse_output_subfolder = output_subfolder_level * ['..']
        reverse_output_subfolder = os.path.join(*reverse_output_subfolder)
        path = os.path.join(reverse_output_subfolder, path)
    # append generator specific output folder
    if 'doxygen_output_folder' in tag_pair:
        path = os.path.join(path, tag_pair['doxygen_output_folder'])
    return path


def package_doxygen_template(template, rd_config, path, package, html_dir, header_filename, footer_filename, manifest_dir, tagfile_dir, output_subfolder):
    # set defaults for overridable keys
    file_patterns = '*.c *.cpp *.h *.cc *.hh *.hpp *.py *.dox *.java'
    excludes = '%s/build/' % path
    predefined = ''

    #Read tagfiles from configuration rather than hardcoding
    tagfiles = ""
    print(rd_config)
    if 'tagfile_spec' in rd_config:
        tagfiles = prepare_tagfiles(rd_config['tagfile_spec'], tagfile_dir, output_subfolder)

    generate_tagfile = ''
    if 'generate_tagfile' in rd_config:
        generate_tagfile = rd_config['generate_tagfile']
        generate_dir = os.path.dirname(generate_tagfile)
        #Make sure that the directory to place the tagfile in exists
        if not os.path.isdir(generate_dir):
            os.makedirs(generate_dir)

    print("Generated the following tagfile string %s" % tagfiles)

    # example path is where htmlinclude operates, so we'll set it to the directory storying manifest.html
    dvars = { '$ALIASES': rd_config.get('aliases', ''),
              '$EXAMPLE_PATH': "%s %s" % (path, manifest_dir),
              '$EXAMPLE_PATTERNS': rd_config.get('example_patterns', ''),
              '$EXCLUDE_PATTERNS': rd_config.get('exclude_patterns', ''),
              '$EXCLUDE_PROP': rd_config.get('exclude', excludes),
              '$EXCLUDE_SYMBOLS': rd_config.get('exclude_symbols', ''),
              '$EXTRACT_ALL': rd_config.get('extract_all', 'YES'),
              '$FILE_PATTERNS': rd_config.get('file_patterns', file_patterns),
              '$GENERATE_TAGFILE': generate_tagfile,
              '$GENERATE_QHP': rd_config.get('generate_qhp', 'NO'),
              '$HTML_FOOTER': footer_filename,
              '$HTML_HEADER': header_filename,
              '$HTML_OUTPUT': os.path.realpath(html_dir),
              '$IMAGE_PATH': rd_config.get('image_path', path), #default to $INPUT
              '$INPUT':  path, '$PROJECT_NAME': package,
              '$JAVADOC_AUTOBRIEF': rd_config.get('javadoc_autobrief', 'NO'),
              '$MULTILINE_CPP_IS_BRIEF': rd_config.get('multiline_cpp_is_brief', 'NO'),
              '$OUTPUT_DIRECTORY': html_dir,
              '$PREDEFINED': rd_config.get('predefined', predefined),
              '$QHP_NAMESPACE': 'org.ros.' + package,
              '$SEARCHENGINE': rd_config.get('searchengine', 'NO'),
              '$TAB_SIZE': rd_config.get('tab_size', '8'),
              '$TAGFILES': tagfiles,
              '$USE_MDFILE_AS_MAINPAGE': rd_config.get('use_mdfile_as_mainpage', '')
              }
    return rdcore.instantiate_template(template, dvars)


def generate_doxygen(path, package, manifest, rd_config, output_dir, quiet):
    """
    Main entrypoint into creating Doxygen documentation
    Will throw an exception if documentation generation fails
    """
    #make sure that we create docs in a subdirectory if requested
    output_subfolder = rd_config.get('output_dir', '.')
    html_dir = os.path.join(output_dir, output_subfolder)

    #Storage for our tempfiles
    files = []
    #We need a temp directory to be able to include the manifest file
    manifest_dir = tempfile.mkdtemp(prefix='rosdoc_lite_doxygen')
    tagfile_dir = tempfile.mkdtemp(prefix='rosdoc_lite_doxygen_tagfiles')
    try:
        if not os.path.isdir(html_dir):
            os.makedirs(html_dir)

        #Create files to write for doxygen generation, these files will be used
        #by doxygen from the command line
        header_file = tempfile.NamedTemporaryFile('w+')
        footer_file = tempfile.NamedTemporaryFile('w+')
        manifest_file = open(os.path.join(manifest_dir, 'manifest.html'), 'w')
        doxygen_file = tempfile.NamedTemporaryFile('w+')
        files = [header_file, footer_file, manifest_file, doxygen_file]

        #Generate our Doxygen templates and fill them in with the right info
        doxy_template = rdcore.load_tmpl('doxy.template')
        doxy = package_doxygen_template(doxy_template, rd_config, path, package, html_dir, header_file.name, footer_file.name, manifest_dir, tagfile_dir, output_subfolder)

        #Throw in manifest infomation into our including templates
        header_template = rdcore.load_tmpl('header.html')
        footer_template = rdcore.load_tmpl('footer.html')
        manifest_template = rdcore.load_tmpl('manifest.html')
        manifest_vars = load_manifest_vars(rd_config, package, manifest)
        header, footer, manifest_html = [rdcore.instantiate_template(t, manifest_vars) for t in [header_template, footer_template, manifest_template]]

        #Actually write files to disk so that doxygen can find them and use them
        for f, tmpl in zip(files, [header, footer, manifest_html, doxy]):
            write_to_file(f, tmpl)

        # doxygenate
        run_doxygen(package, doxygen_file.name, quiet)

        """
        # support files (stylesheets)
        # Can uncomment this to get old ROS styles for doxygen, but I prefer the defaults
        # I just think they look better
        dstyles_in = os.path.join(rdcore.get_templates_dir(), 'doxygen.css')
        dstyles_css = os.path.join(html_dir, 'doxygen.css')
        shutil.copyfile(dstyles_in, dstyles_css)
        """

    except Exception, e:
        print("ERROR: Doxygen of package [%s] failed: %s" % (package, str(e)), file=sys.stderr)
        #make sure to pass the exception up the stack
        raise
    finally:
        #make sure to clean up
        for f in files:
            f.close()
        shutil.rmtree(manifest_dir)
        shutil.rmtree(tagfile_dir)
