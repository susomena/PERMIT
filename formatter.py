# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Jesús Mena-Oreja <jmena@umh.es>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import argparse


class CustomFormatter(argparse.HelpFormatter):
    """
    Class to make the help metavars appear only once in the simulator help
    and make the help text of each argument appear more to the right (in the
    style of the SUMO help).
    Code for the _format_action_invocation function obtained from the
    accepted answer in:
    https://stackoverflow.com/questions/23936145/python-argparse-help-message-disable-metavar-for-short-options
    """

    def __init__(self, prog, indent_increment=2, max_help_position=49, width=None, min_help_position=39):
        super(CustomFormatter, self).__init__(prog, indent_increment, max_help_position, width)
        self._min_help_position = min_help_position

    def _format_action(self, action):
        # determine the required width and the entry label
        # help_position = min(self._action_max_length + 2, self._max_help_position)
        action_header = self._format_action_invocation(action)
        help_position = max(len(action_header) + 4, self._min_help_position)
        help_position = min(help_position, self._max_help_position)
        help_width = max(self._width - help_position, 11)
        action_width = help_position - self._current_indent - 2

        # no help; start on same line and add a final newline
        if not action.help:
            tup = self._current_indent, '', action_header
            action_header = '%*s%s\n' % tup

        # short action name; start on the same line and pad two spaces
        elif len(action_header) <= action_width:
            tup = self._current_indent, '', action_width, action_header
            action_header = '%*s%-*s  ' % tup
            indent_first = 0

        # long action name; start on the next line
        else:
            tup = self._current_indent, '', action_header
            action_header = '%*s%s\n' % tup
            indent_first = help_position

        # collect the pieces of the action help
        parts = [action_header]

        # if there was help for the action, add lines of help text
        if action.help:
            help_text = self._expand_help(action)
            help_lines = self._split_lines(help_text, help_width)
            parts.append('%*s%s\n' % (indent_first, '', help_lines[0]))
            for line in help_lines[1:]:
                # parts.append('%*s%s\n' % (help_position, '', line))
                parts.append('%*s%s\n' % (self._min_help_position, '', line))

        # or add a newline if the description doesn't end with one
        elif not action_header.endswith('\n'):
            parts.append('\n')

        # if there are any sub-actions, add their help as well
        for subaction in self._iter_indented_subactions(action):
            parts.append(self._format_action(subaction))

        # return a single string
        return self._join_parts(parts)

    def _format_action_invocation(self, action):
        if not action.option_strings:
            metavar, = self._metavar_formatter(action, action.dest)(1)
            return metavar
        else:
            parts = []
            # if the Optional doesn't take a value, format is:
            #    -s, --long
            if action.nargs == 0:
                parts.extend(action.option_strings)

            # if the Optional takes a value, format is:
            #    -s ARGS, --long ARGS
            # change to:
            #    -s, --long ARGS
            else:
                default = action.dest.upper()
                args_string = self._format_args(action, default)
                for option_string in action.option_strings:
                    # parts.append('%s %s' % (option_string, args_string))
                    parts.append('%s' % option_string)
                parts[-1] += ' %s' % args_string
            return ', '.join(parts)
