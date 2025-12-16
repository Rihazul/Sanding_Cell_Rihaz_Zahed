import argparse
import sys
import time

from minimalmodbus import InvalidResponseError, NoResponseError

from modules.final_async_reader import getInstrument, getRawHeight, scale_value


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Poll the laser scanner and print values in a tight loop."
    )
    parser.add_argument(
        "--port",
        help="Serial port for the laser (defaults to LASER_COM_PORT env or COM8).",
    )
    parser.add_argument(
        "--slave",
        type=int,
        help="Modbus slave id (defaults to LASER_SLAVE env or 1).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        help="Serial baudrate (defaults to LASER_BAUD env or 115200).",
    )
    parser.add_argument(
        "--parity",
        help="Serial parity (N/E/O, defaults to LASER_PARITY env or N).",
    )
    parser.add_argument(
        "--stopbits",
        type=int,
        help="Serial stopbits (defaults to LASER_STOPBITS env or 1).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        help="Serial read/write timeout seconds (defaults to LASER_TIMEOUT env or 1.0).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Seconds between reads.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    instrument = getInstrument(
    )

    try:
        while True:
            raw = getRawHeight(instrument)
            print(f"Laser port={instrument.serial.port} raw={raw:.3f}", end="\r", flush=True)
            scaled = scale_value(raw)
            print(
                f"Laser port={instrument.serial.port} raw={raw:.3f} scaled={scaled:.2f}",
                end="\r",
                flush=True,
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except (NoResponseError, InvalidResponseError) as exc:
        print(f"\nLaser read error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            instrument.serial.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
