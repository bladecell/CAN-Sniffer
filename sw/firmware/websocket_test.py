import argparse
import asyncio
import websockets
import requests
import signal
import sys
from datetime import datetime

from rich import print
from rich.console import Console

# --- CONFIGURATION ---
ESP_IP = "can-sniffer.local"
WS_URL = f"ws://{ESP_IP}/ws"
CAN_SNIFFER_URL = f"http://{ESP_IP}"

console = Console()


def start_pid_poll():
    try:
        response = requests.post(
            f"{CAN_SNIFFER_URL}/api/v1/req/pid_poll?running=true", timeout=5
        )
        console.print(
            f"[bold cyan]PID poll response:[/] [green]{response.status_code}[/]"
        )
    except requests.exceptions.RequestException as e:
        console.print(f"[bold red]PID poll failed:[/] {e}")
        sys.exit(1)


async def debug_stream():
    console.print(f"[bold yellow]Connecting to[/] [underline]{WS_URL}[/]...")

    async with websockets.connect(WS_URL) as ws:
        console.print("[bold green]Connected![/]")
        await ws.send(bytes([0xA0]))
        console.print("[bold cyan]Sent Start Command. Waiting for data...[/]\n")

        async for data in ws:
            if not isinstance(data, bytes):
                print(data)
                continue

            if len(data) < 2:
                print(f"[bold red]Too short:[/] [dim]{data.hex()}[/]")
                continue

            mode = data[0]
            length = data[1]
            payload = data[2 : 2 + length]

            hex_str = " ".join(f"{b:02X}" for b in payload)
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            print(
                f"[dim]{ts}[/] | "
                f"[bold white]Mode:[/] [cyan]0x{mode:02X}[/] | "
                f"[bold white]Len:[/] [yellow]{length}[/] | "
                f"[bold white]Data:[/] [green]{hex_str}[/]"
            )


def handle_exit(sig, frame):
    console.print("\n[bold yellow]Stopped.[/]")
    sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CAN sniffer WebSocket logger")
    parser.add_argument(
        "--poll", action="store_true", help="Enable PID poll before connecting"
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, handle_exit)

    if args.poll:
        start_pid_poll()
    else:
        console.print("[dim]PID poll skipped (use --poll to enable)[/]")

    try:
        asyncio.run(debug_stream())
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Stopped.[/]")
