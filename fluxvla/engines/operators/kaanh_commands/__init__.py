"""Local commands: add each command module here to expose it in the console."""

from . import m_init, stop, set_acc


LOCAL_COMMANDS = {'m_init': m_init, 'stop': stop, 'set_acc': set_acc}


def add_local_command_parsers(subparsers):
    for name, command in LOCAL_COMMANDS.items():
        parser = subparsers.add_parser(name, add_help=False, allow_abbrev=False)
        command.configure_parser(parser)
