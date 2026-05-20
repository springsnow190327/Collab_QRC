#!/usr/bin/python3

import sys
import os

# Modify PYTHONPATH so we can obtain the version data from setup module.
# pylint: disable=wrong-import-position
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'nvblox_torch')))
from setup import get_expected_wheel_filename

import argparse

import subprocess
import tempfile
import requests

# These artifactory URLs are used for internal publishing of the nvblox_torch wheel.
# Browseable URL: https://urm.nvidia.com/ui/repos/tree/General/hw-nvblox-alpine-local/pypi
# Note that the repo is not pypi compliant. This means that have provide the complete URL
# to the .whl file when installing using pip.
REMOTE_URL_BASE = 'https://urm.nvidia.com/artifactory/hw-nvblox-alpine/pypi/'
STAGING_URL_BASE = f'{REMOTE_URL_BASE}staging/nvblox_torch/'
RELEASE_URL_BASE = f'{REMOTE_URL_BASE}release/nvblox_torch/'


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='''Deployment script for nvblox_torch.
        Contains functionality for handling naming and versioning + internal publishing of the nvblox_torch wheel.
        The intended release flow is
          1. Run this script with "--publish-to-staging" to push the wheel to a temprary staging area.
          2. Pip install the wheel from the staging area in a bare test environment. The stagingURL can be obtained by --print-staging-url.
          3. Run tests in the test environment.
          4. Run this script with "--publish-to-release" to ship the thing to a permanent release area.
          5. Profit.

          The deployed wheel will have a PEP345 compliant filename:
             nvblox_torch-<version_string>+cu<X>ubuntu<Y>-0<build_number>-py3-none-linux_x86_64.whl

          Where:
            <version_string> is the version of the wheel as defined in setup.py
            <X> is the major CUDA version
            <Y> is the major Ubuntu version
            <build_number> CI/CD build number.

          For example: nvblox_torch-0.0.1+cu12ubuntu24-02ec710-py3-none-linux_x86_64.whl
          ''')
    parser.add_argument('--publish-to-staging',
                        action='store_true',
                        help='Build and publish to staging area.')
    parser.add_argument('--publish-to-release',
                        action='store_true',
                        help='Build and publish to release.')
    parser.add_argument('--print-staging-url', action='store_true', help='Print the staging URL')
    parser.add_argument('--print-release-url', action='store_true', help='Print the release URL')
    parser.add_argument('--build-package', action='store_true', help='Build the package')
    parser.add_argument('--build-number', type=str, required=True, help='Build number')
    parser.add_argument('--username', type=str, required=False, help='Username for artifactory')
    parser.add_argument(
        '--password',
        type=str,
        required=False,
        help='Artifactory API key (urm.nvidia.com -> user menu -> user profile -> generate token')
    return parser.parse_args()


def get_release_url(build_number: str) -> str:
    """Get the URL for the release of the current version/build of nvblox_torch."""
    return RELEASE_URL_BASE + get_expected_wheel_filename(build_number)


def get_staging_url(build_number: str) -> str:
    """Get the URL for the staging of the current version/build of nvblox_torch."""
    return STAGING_URL_BASE + get_expected_wheel_filename(build_number)


def get_local_wheel_path(build_number: str) -> str:
    """Get the path to the local wheel file for the current version/build of nvblox_torch."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    wheel_path = os.path.join(script_dir, '..', 'nvblox_torch', 'dist',
                              get_expected_wheel_filename(build_number))
    return wheel_path


def publish(url: str, username: str, password: str, build_number: str) -> None:
    """Publish the version/build of nvblox_torch to the given URL."""

    assert username is not None and password is not None, \
        'Username and password are required when publishing to artifactory'

    build_package(build_number)

    # Ship it!
    curl_cmd = f'curl --verbose --upload-file {get_local_wheel_path(build_number)} {url}'
    curl_cmd += f' --user {username}:{password}'
    subprocess.run(curl_cmd, shell=True, check=True)

    # Validate
    validate_published_file(url, build_number, username, password)


def build_package(build_number: str) -> None:
    """Build the package if it doesn't exist."""
    local_wheel_path = get_local_wheel_path(build_number)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    nvblox_torch_dir = os.path.join(script_dir, '..', 'nvblox_torch')

    # Delete the wheel file if it already exists
    if os.path.exists(local_wheel_path):
        os.remove(local_wheel_path)

    subprocess.run([
        'python', 'setup.py', 'bdist_wheel', '--plat-name', 'linux_x86_64', '--build-number',
        build_number
    ],
                   check=True,
                   cwd=nvblox_torch_dir)

    assert os.path.exists(local_wheel_path), f'Wheel file was not created: {local_wheel_path}'


def validate_published_file(published_url: str, build_number: str, username: str,
                            password: str) -> None:
    """Validate the published file by downloading it and comparing the md5sum to
    the expected value."""

    print(f'Downloading and validating published file: {published_url}')
    with tempfile.TemporaryDirectory() as tmp_dir:
        downloaded_file = os.path.join(tmp_dir, 'downloaded.whl')
        response = requests.get(published_url, auth=(username, password))
        response.raise_for_status()
        with open(downloaded_file, 'wb') as f:
            f.write(response.content)

        # Calculate md5sums using subprocess
        local_md5 = subprocess.check_output(['md5sum', get_local_wheel_path(build_number)
                                             ]).decode().split()[0]
        downloaded_md5 = subprocess.check_output(['md5sum', downloaded_file]).decode().split()[0]

        assert local_md5 == downloaded_md5, (
            f'MD5 sum mismatch between local ({local_md5}) and published ({downloaded_md5}) files')
        print('Validation successful')


def main(args: argparse.Namespace) -> int:
    staging_url = get_staging_url(args.build_number)
    release_url = get_release_url(args.build_number)
    if args.print_staging_url:
        print(staging_url)
    elif args.print_release_url:
        print(release_url)
    elif args.build_package:
        build_package(args.build_number)
        print('Finished building package')
    elif args.publish_to_staging:
        publish(staging_url, args.username, args.password, args.build_number)
        print('Finished publishing to staging')
    elif args.publish_to_release:
        publish(release_url, args.username, args.password, args.build_number)
        print('Finished publishing to release')
    else:
        print('Nothing to do')
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main(parse_args()))
