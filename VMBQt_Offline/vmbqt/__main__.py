import argparse
import json
from dataclasses import asdict
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description="VMBQt: offline Qt6 Lanelet2 editor")
    parser.add_argument("file",nargs="?",help=".vmbqt project or local-coordinate .osm")
    parser.add_argument("--validate",metavar="OSM",help="run internal checks without a display")
    parser.add_argument("--projection",metavar="YAML",help="projection used when validating geographic OSM")
    args=parser.parse_args()
    if args.validate:
        from .model import Document
        from .projection import ProjectionConfig
        from .validation import validate
        config=ProjectionConfig.from_yaml(args.projection) if args.projection else ProjectionConfig()
        issues=validate(Document.read_osm(args.validate,config))
        print(json.dumps([asdict(i) for i in issues],ensure_ascii=False,indent=2))
        return 1 if any(i.level=="ERROR" for i in issues) else 0
    from .app import run
    return run(args.file)


if __name__=="__main__":
    raise SystemExit(main())
