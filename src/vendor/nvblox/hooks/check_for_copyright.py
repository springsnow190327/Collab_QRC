#!/usr/bin/env python

from typing import List
import sys
import re

# Pattern for the first copyright header
pattern_nvidia_copyright = r"""
Copyright \(c\) \d{4}(?:-\d{4})?, NVIDIA CORPORATION\. All rights reserved\.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto\. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited\.
"""

# Pattern for the Apache License header
pattern_apache_license = r"""
/\*
Copyright \d{4} NVIDIA CORPORATION

Licensed under the Apache License, Version 2\.0 \(the "License"\);
you may not use this file except in compliance with the License\.
You may obtain a copy of the License at

\s+http://www\.apache\.org/licenses/LICENSE-2\.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied\.
See the License for the specific language governing permissions and
limitations under the License\.
\*/
"""

# Combine the two patterns using alternation
combined_pattern = f'(?:{pattern_nvidia_copyright})|(?:{pattern_apache_license})'

pattern = re.compile(re.escape(combined_pattern), re.MULTILINE | re.DOTALL | re.VERBOSE)


def check_copyright(files: List[str]) -> int:
    files_missing_copyright = []
    for file in files:
        with open(file, 'r', encoding='utf-8') as f:
            content = f.read()
            if not pattern.search(content):
                files_missing_copyright.append(file)

    if files_missing_copyright:
        for file in files_missing_copyright:
            print(f'The following file is missing a copyright: {file}')
        return 1
    return 0


def main() -> None:
    files = sys.argv[1:]
    sys.exit(check_copyright(files))


if __name__ == '__main__':
    main()
