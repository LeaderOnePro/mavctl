"""CLI layer — thin Typer shell.

Parses arguments, calls the daemon, formats output, and maps daemon errors
to process exit codes. Contains no business logic; never imports pymavlink
or the adapter layer.
"""

from mavctl.cli.app import main

__all__ = ["main"]
