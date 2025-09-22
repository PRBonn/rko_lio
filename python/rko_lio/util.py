from rich.console import Console
from rich.panel import Panel

console = Console()


def error(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(msg, title="Error", border_style="red", expand=False, title_align="left")
    )


def warning(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(
            msg,
            title="Warning",
            border_style="yellow",
            expand=False,
            title_align="left",
        )
    )


def info(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(msg, title="Info", border_style="cyan", expand=False, title_align="left")
    )
