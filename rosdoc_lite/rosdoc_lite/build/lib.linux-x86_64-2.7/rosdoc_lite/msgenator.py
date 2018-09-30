# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import cStringIO
import os
import sys
import time

from . import rdcore
import genmsg
import fnmatch


def _find_files_with_extension(path, ext):
    matches = []
    for root, dirnames, filenames in os.walk(path):
        for filename in fnmatch.filter(filenames, '*.%s' % ext):
            #matches.append(os.path.join(root, filename))
            matches.append((os.path.splitext(filename)[0], os.path.join(root, filename)))
    return matches


def _href(link, text):
    return '<a href="%(link)s">%(text)s</a>' % locals()


def resource_name(resource):
    if '/' in resource:
        val = tuple(resource.split('/'))
        if len(val) != 2:
            raise ValueError("invalid name [%s]" % resource)
        else:
            return val
    else:
        return '', resource


def type_link(type_, base_package):
    """
    :param type_: str
    :param base_package: containing package
    :returns: A link to the page of the type, or just a label for basetypes
    """
    base_type = genmsg.msgs.parse_type(type_)[0]
    base_type = genmsg.msgs.resolve_type(base_type, base_package)
    if base_type in genmsg.msgs.BUILTIN_TYPES:
        return type_
    package, base_type = resource_name(base_type)
    # always chain upwards to msg dir
    return _href("../../../%(package)s/html/msg/%(base_type)s.html" % locals(), type_)


def index_type_link(pref, type_, base_package):
    if type_ in genmsg.msgs.BUILTIN_TYPES:
        return type_
    base_type_ = genmsg.msgs.parse_type(type_)[0]
    package, base_type_ = resource_name(base_type_)
    if not package or package == base_package:
        return _href("%s/%s.html" % (pref, base_type_), type_)
    else:
        return _href("../../%(package)s/html/%(pref)s/%(base_type_)s.html" % locals(), type_)

def _generate_raw_text(raw_text):
    s = ''
    for line in raw_text.split('\n'):
        line = line.replace(' ', '&nbsp;')
        parts = line.split('#')
        if len(parts) > 1:
            s = s + parts[0]+'<font color="blue">#%s</font><br/>'%('#'.join(parts[1:]))
        else:
            s = s + "%s<br/>"%parts[0]
    return s

def _generate_msg_text_from_spec(package, spec, msg_context, buff=None, indent=0):
    if buff is None:
        buff = cStringIO.StringIO()
    for c in spec.constants:
        buff.write("%s%s %s=%s<br />" % ("&nbsp;"*indent, c.type, c.name, c.val_text))
    for type_, name in zip(spec.types, spec.names):
        buff.write("%s%s %s<br />" % ("&nbsp;"*indent, type_link(type_, package), name))
        base_type = genmsg.msgs.parse_type(type_)[0]
        base_type = genmsg.msgs.resolve_type(base_type, package)
        #TODO: If you actually want an expanded definition, you need some way
        # to go from package name to message type, with the new catkin stuff,
        #I'm not sure this is possible
        #if not base_type in genmsg.msgs.BUILTIN_TYPES:
        #    subspec = None
        #    if not msg_context.is_registered(base_type):
        #        package, msg_type = resource_name(base_type)
        #        path = os.path.join(rp.get_path(package), 'msg', "%s.msg" % msg_type)
        #        subspec = genmsg.msg_loader.load_msg_from_file(msg_context, path, "%s/%s" % (package, msg_type))
        #    else:
        #        subspec = msg_context.get_registered(base_type)
        #    _generate_msg_text_from_spec(package, subspec, msg_context, rp, buff, indent + 4)
    return buff.getvalue()


def _generate_msg_text(package, type_, msg_context, spec):
    #print("generate", package, type_)
    return _generate_msg_text_from_spec(package, spec, msg_context)


def _generate_srv_text(package, type_, msg_context, spec):
    return _generate_msg_text_from_spec(package, spec.request, msg_context) + \
        '<hr />' + \
        _generate_msg_text_from_spec(package, spec.response, msg_context)

def generate_action_doc(action, action_template, path):
    package, base_type = resource_name(action)
    print ("action: %s" % action)
    d = {'name': action, 'ext': 'action', 'type': 'Action',
         'package': package, 'base_type': base_type,
         'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    with open(path, 'r') as f:
        raw_text = f.read()
    d['raw_text'] = _generate_raw_text(raw_text)
    return action_template % d

def generate_srv_doc(srv, msg_context, msg_template, path):
    package, base_type = resource_name(srv)
    print("srv: %s" % srv)
    d = {'name': srv, 'ext': 'srv', 'type': 'Service',
         'package': package, 'base_type': base_type,
         'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    spec = genmsg.msg_loader.load_srv_from_file(msg_context, path, "%s/%s" % (package, base_type))
    d['fancy_text'] = _generate_srv_text(package, base_type, msg_context, spec)
    d['raw_text'] = _generate_raw_text(spec.text)
    return msg_template % d


def generate_msg_doc(msg, msg_context, msg_template, path):
    package, base_type = resource_name(msg)
    d = {'name': msg, 'ext': 'msg', 'type': 'Message',
         'package': package, 'base_type': base_type,
         'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    spec = genmsg.msg_loader.load_msg_from_file(msg_context, path, "%s/%s" % (package, base_type))
    d['fancy_text'] = _generate_msg_text(package, base_type, msg_context, spec)
    d['raw_text'] = _generate_raw_text(spec.text)
    return msg_template % d


def generate_msg_index(package, file_d, msgs, srvs, actions, wiki_url, msg_index_template):
    d = {'package': package, 'msg_list': '', 'srv_list': '', 'action_list': '',
         'package_url': wiki_url,
         'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    if msgs:
        d['msg_list'] = """<h2>Message types</h2>
<div class="msg-list">
  <ul>
%s
  </ul>
</div>""" % '\n'.join([" <li>%s</li>" % index_type_link('msg', m, package) for m in sorted(msgs)])

    if srvs:
        d['srv_list'] = """<h2>Service types</h2>
<div class="srv-list">
  <ul>
%s
  </ul>
</div>""" % '\n'.join([" <li>%s</li>" % index_type_link('srv', s, package) for s in sorted(srvs)])

    if actions:
        d['action_list'] = """<h2>Action types</h2>
<div class="action-list">
  <ul>
%s
  </ul>
</div>""" % '\n'.join([" <li>%s</li>" % index_type_link('action', a, package) for a in sorted(actions)])

    file_p = os.path.join(file_d, 'index-msg.html')
    text = msg_index_template % d
    with open(file_p, 'w') as f:
        #print("writing", file_p)
        f.write(text)


def generate_msg_docs(package, path, manifest, output_dir):
    """generate manifest.yaml files for MoinMoin PackageHeader macro"""
    try:
        import yaml
    except ImportError:
        print("Cannot import yaml, will not generate MoinMoin PackageHeader files", file=sys.stderr)
        return

    # create the directory for the autogen files
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # get a list of what we are documenting
    msgs = _find_files_with_extension(path, 'msg')
    srvs = _find_files_with_extension(path, 'srv')
    actions = _find_files_with_extension(path, 'action')

    print(msgs)
    print(srvs)
    print(actions)

    #Load template files
    msg_template = rdcore.load_tmpl('msg.template')
    action_template = rdcore.load_tmpl('action.template')
    msg_index_template = rdcore.load_tmpl('msg-index.template')

    msg_success = []
    srv_success = []
    action_success = []

    # create dir for msg documentation
    if msgs:
        msg_d = os.path.join(output_dir, 'msg')
        if not os.path.exists(msg_d):
            os.makedirs(msg_d)

    # document the messages
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    for m, msg_path in msgs:
        try:
            text = generate_msg_doc('%s/%s'%(package,m), msg_context, msg_template, msg_path)
            file_p = os.path.join(msg_d, '%s.html' % m)
            with open(file_p, 'w') as f:
                #print("writing", file_p)
                f.write(text)
            msg_success.append(m)
        except Exception, e:
            print("FAILED to generate for %s/%s: %s" % (package, m, str(e)), file=sys.stderr)

    # create dir for srv documentation
    if srvs:
        srv_d = os.path.join(output_dir, 'srv')
        if not os.path.exists(srv_d):
            os.makedirs(srv_d)

    # document the services
    for s, srv_path in srvs:
        try:
            text = generate_srv_doc('%s/%s' % (package,s), msg_context, msg_template, srv_path)
            file_p = os.path.join(srv_d, '%s.html' % s)
            with open(file_p, 'w') as f:
                #print("writing", file_p)
                f.write(text)
            srv_success.append(s)
        except Exception, e:
            print("FAILED to generate for %s/%s: %s" % (package, s, str(e)), file=sys.stderr)
            raise

    #create dir for action documentation
    if actions:
        action_d = os.path.join(output_dir, 'action')
        if not os.path.exists(action_d):
            os.makedirs(action_d)

    #document the actions
    for a, action_path in actions:
        try:
            text = generate_action_doc('%s/%s' % (package,a), action_template, action_path)
            file_p = os.path.join(action_d, '%s.html' % a)
            with open(file_p, 'w') as f:
                #print("writing", file_p)
                f.write(text)
            action_success.append(a)
        except Exception, e:
            print("FAILED to generate for %s/%s: %s" % (package, a, str(e)), file=sys.stderr)
            raise

    # generate the top-level index
    website_url = '<li>%s</li>\n' % _href(manifest.url, 'Website')
    generate_msg_index(package, output_dir, msg_success, srv_success, action_success, website_url, msg_index_template)

    return (msg_success, srv_success, action_success)
