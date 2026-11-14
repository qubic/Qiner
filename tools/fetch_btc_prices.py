#!/usr/bin/env python3
"""Fetch the BTC/USDT prices of the node's bootstrap price series and write them as '<hour>,<price>' lines.

The price of hour H is the last Binance BTCUSDT trade before H, which is the close of the 1h kline that
opens one hour earlier. Hour 0 starts at 2022-04-13 12:00 UTC, so epoch e starts at hour 168 * e.

"Hour" is one step of the series. --period-sec sets its length and must equal the node's BTC_FEED_PERIOD_SEC
(3600 on mainnet). With another period the prices are the closes of 1-minute klines.

Usage:
    python3 fetch_btc_prices.py --out btc_prices.csv
    python3 fetch_btc_prices.py --period-sec 120 --out btc_prices.csv
    python3 fetch_btc_prices.py --next-hour 41100 --out btc_prices.csv
    python3 fetch_btc_prices.py --dummy --period-sec 120 --out btc_prices.csv

--dummy writes formula prices instead of calling Binance, for test networks whose oracle machines answer
with the same formula.

Next step: btc_price_bootstrap <btc_prices.csv> <epoch> writes btc_price_bootstrap.bin.
Needs only the Python standard library.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

HOUR_ZERO_MS: int = 1649851200000  # 2022-04-13 12:00:00 UTC
KLINES_URL: str = "https://api.binance.com/api/v3/klines?symbol=BTCUSDT&interval={interval}&startTime={start}&limit=1000"
MIN_POINTS: int = 24 * 365 + 24 * 7 + 1
MASK64: int = (1 << 64) - 1


@dataclass(frozen=True)
class HourPrice:
    hour: int
    price: str


def checked_price(text: str) -> str:
    """The exchange's decimal string, unchanged; at most 8 decimals."""
    whole, _, frac = text.strip().partition(".")
    if not whole.isdigit() or (frac and not frac.isdigit()) or len(frac) > 8:
        raise SystemExit(f"not a price with at most 8 decimals: {text!r}")
    return text.strip()


def dummy_price(hour: int, hour_ms: int, pair: str = "BTC/USDT") -> str:
    """Formula price of the hour: 69000.00 .. 71000.00, from the pair and the minute before the hour."""
    fnv = 0xCBF29CE484222325
    for byte in pair.encode("ascii"):
        fnv = ((fnv ^ byte) * 0x100000001B3) & MASK64
    minute = (HOUR_ZERO_MS + hour * hour_ms) // 60000 - 1
    z = (minute + fnv + 0x9E3779B97F4A7C15) & MASK64
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    z ^= z >> 31
    cents = 7000000 + z % 200001 - 100000
    return f"{cents // 100}.{cents % 100:02d}"


def fetch_prices(first_hour: int, last_hour: int, hour_ms: int) -> list[HourPrice]:
    # The price of a step is the close of the kline that ends at the step: the 1h kline for one-hour
    # steps, otherwise the 1-minute kline that opens one minute before the step.
    kline_ms = hour_ms if hour_ms == 3600000 else 60000
    interval = "1h" if kline_ms == 3600000 else "1m"
    wanted = {HOUR_ZERO_MS + hour * hour_ms - kline_ms: hour for hour in range(first_hour, last_hour + 1)}
    rows: list[HourPrice] = []
    start_ms = HOUR_ZERO_MS + first_hour * hour_ms - kline_ms
    end_ms = HOUR_ZERO_MS + last_hour * hour_ms - kline_ms
    while start_ms <= end_ms:
        request = urllib.request.Request(KLINES_URL.format(interval=interval, start=start_ms), headers={"User-Agent": "btc-price-bootstrap"})
        klines = None
        for attempt in range(4):
            try:
                with urllib.request.urlopen(request, timeout=30) as response:
                    klines = json.loads(response.read())
                break
            except (urllib.error.URLError, TimeoutError, OSError) as error:
                if attempt == 3:
                    raise SystemExit(f"Binance request failed: {error}") from error
                time.sleep(5)
        if not klines:
            break
        for kline in klines:
            open_ms = int(kline[0])
            if open_ms > end_ms:
                break
            if open_ms != start_ms:
                raise SystemExit(f"gap in klines: expected open time {start_ms}, got {open_ms}")
            if open_ms in wanted:
                rows.append(HourPrice(wanted[open_ms], checked_price(str(kline[4]))))
            start_ms += kline_ms
        time.sleep(0.2)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--next-hour", type=int, help="first hour NOT in the file; default: the hour running now")
    parser.add_argument("--period-sec", type=int, default=3600, help="length of one step; must equal the node's BTC_FEED_PERIOD_SEC")
    parser.add_argument("--dummy", action="store_true", help="formula prices instead of Binance, for test networks")
    parser.add_argument("--out", type=Path, default=Path("btc_prices.csv"))
    args = parser.parse_args()
    if args.period_sec < 60 or args.period_sec % 60 != 0:
        parser.error("--period-sec must be a multiple of 60")
    hour_ms: int = args.period_sec * 1000

    next_hour = args.next_hour
    if next_hour is None:
        # The hour that started most recently is complete (its price is the trade before it).
        next_hour = (int(time.time() * 1000) - HOUR_ZERO_MS) // hour_ms + 1
    first_hour = next_hour - MIN_POINTS
    if first_hour < 0:
        parser.error(f"--next-hour must be at least {MIN_POINTS}")

    if args.dummy:
        rows = [HourPrice(hour, dummy_price(hour, hour_ms)) for hour in range(first_hour, next_hour)]
    else:
        rows = fetch_prices(first_hour, next_hour - 1, hour_ms)
    if [row.hour for row in rows] != list(range(first_hour, next_hour)):
        raise SystemExit(f"got {len(rows)} prices, expected the {MIN_POINTS} hours {first_hour} .. {next_hour - 1}")

    args.out.write_text("".join(f"{row.hour},{row.price}\n" for row in rows))
    print(f"wrote {args.out}: hours {first_hour} .. {next_hour - 1} ({len(rows)} prices)")


if __name__ == "__main__":
    main()
