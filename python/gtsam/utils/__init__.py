import os
import shutil
import urllib.error
import urllib.request
from pathlib import Path

# Location of the example data files in the GTSAM repository, used when the
# data is not bundled with the package (e.g. wheels from PyPI).
EXAMPLE_DATA_URL = "https://raw.githubusercontent.com/borglab/gtsam/develop/examples/Data"

_EXTENSIONS = ("", ".graph", ".txt", ".out", ".xml", ".g2o")


def exampleDataCacheDir():
    """
    Directory where downloaded example data files are cached.

    Can be overridden with the `GTSAM_DATA_CACHE_DIR` environment variable.
    """
    if "GTSAM_DATA_CACHE_DIR" in os.environ:
        return Path(os.environ["GTSAM_DATA_CACHE_DIR"])
    cache_home = os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    return Path(cache_home) / "gtsam" / "Data"


def _localDataRoots():
    """Example data directories available on this machine, in search order."""
    import gtsam

    package_path = Path(gtsam.__path__[0]).resolve()
    roots_to_search = []

    if "GTSAM_EXAMPLE_DATA_DIR" in os.environ:
        roots_to_search.append(Path(os.environ["GTSAM_EXAMPLE_DATA_DIR"]))

    for parent in package_path.parents:
        for candidate in (
            parent / "examples" / "Data",
            parent / "gtsam_examples" / "Data",
        ):
            roots_to_search.append(candidate)

    roots_to_search.append(package_path / "Data")
    roots_to_search.append(exampleDataCacheDir())

    unique_roots = []
    for root in roots_to_search:
        if root.exists() and root not in unique_roots:
            unique_roots.append(root)
    return unique_roots


def _downloadExampleDataFile(name):
    """Download `name` (trying the usual extensions) into the cache directory."""
    base_url = os.environ.get("GTSAM_EXAMPLE_DATA_URL", EXAMPLE_DATA_URL).rstrip("/")
    cache_dir = exampleDataCacheDir()
    for ext in _EXTENSIONS:
        relative = Path(name + ext).as_posix()
        url = f"{base_url}/{urllib.request.quote(relative)}"
        try:
            with urllib.request.urlopen(url, timeout=60) as response:
                destination = cache_dir / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                partial = destination.with_name(destination.name + ".part")
                with open(partial, "wb") as file:
                    shutil.copyfileobj(response, file)
                partial.replace(destination)
                return str(destination)
        except urllib.error.HTTPError as error:
            if error.code != 404:
                raise
        except (urllib.error.URLError, OSError):
            return None
    return None


def findExampleDataFile(name):
    """
    Find the example data file specified by `name`.

    The search covers `GTSAM_EXAMPLE_DATA_DIR`, the GTSAM source tree, data
    bundled with the package, and the download cache. If the file is not found
    locally, it is downloaded from the GTSAM repository into the cache
    (see `exampleDataCacheDir`). Set `GTSAM_OFFLINE=1` to disable downloads.
    """
    requested = Path(name)
    unique_roots = _localDataRoots()

    for root in unique_roots:
        for ext in _EXTENSIONS:
            candidate = root / (name + ext)
            if candidate.is_file() or candidate.is_dir():
                return str(candidate)

    if len(requested.parts) == 1:
        for root in unique_roots:
            for candidate in root.iterdir():
                for ext in _EXTENSIONS:
                    if candidate.name == name + ext:
                        return str(candidate)

    if os.environ.get("GTSAM_OFFLINE", "0") in ("", "0"):
        downloaded = _downloadExampleDataFile(name)
        if downloaded is not None:
            return downloaded

    raise FileNotFoundError(
        f"Could not find example data file '{name}' under any known GTSAM example data root, "
        f"and it could not be downloaded from {EXAMPLE_DATA_URL}."
    )
