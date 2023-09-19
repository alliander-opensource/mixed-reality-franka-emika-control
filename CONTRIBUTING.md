# How to Contribute

We'd love to accept your patches and contributions to this project. There are
just a few small guidelines you need to follow.

## Common rules

For all contributons to this open source projects

1. Please associate your commit with your Alliander.com email unless you have a history of contributing to the repo under a different email before your employment at Alliander. 
1. You and your team may commit to the project without contacting the project members first, but please contact the project memebers if you are making significant changes to the project’s scope.

## Filing bugs and change requests

You can file bugs against and change requests for the project via github issues. Consult [GitHub Help](https://docs.github.com/en/free-pro-team@latest/github/managing-your-work-on-github/creating-an-issue) for more
information on using github issues.

## Community Guidelines

This project follows the following [Code of Conduct](https://github.com/Alliander/ospo-examples/blob/master/new-open-source-project/docs/Code-of-conduct.md).

## Source Code Headers

Every file containing source code must include copyright. This includes any JS/CSS files that you might be serving out to
browsers. 

Apache 2.0 header: 

     SPDX-FileCopyrightText: 'Copyright Contributors to the [name_open_source_project] project' 
     SPDX-License-Identifier: Apache-2.0

## Git branching

This project uses the [GitHub flow Workflow](https://guides.github.com/introduction/flow/) and branching model. 
The `main` branch always contains the latest release. 
New feature/fix branches are branched from `main`. 
When a feature/fix is finished it is merged back into `main` via a 
[Pull Request](https://docs.github.com/en/github/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests).

In case of major version release with new features and/or breaking changes, we might temporarily create a 
`release/` branch to hold all the changes until they are merged into `main`.

## Code reviews

All patches and contributions, including patches and contributions by project members, require review by one of the maintainers of the project. We
use GitHub pull requests for this purpose. Consult
[GitHub Help](https://help.github.com/articles/about-pull-requests/) for more
information on using pull requests.

## Pull Request Process
Contributions should be submitted as Github pull requests. See [Creating a pull request](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request) if you're unfamiliar with this concept.

The process for a code change and pull request you should follow:

1. Create a topic branch in your local repository, following the naming format
"feature-###" or "fix-###". For more information see the Git branching guideline.
1. Make changes, compile, and test thoroughly. Ensure any install or build dependencies are removed before the end of the layer when doing a build. Code style should match existing style and conventions, and changes should be focused on the topic the pull request will be addressed. For more information see the style guide.
1. Push commits to your fork.
1. Create a Github pull request from your topic branch.
1. Pull requests will be reviewed by one of the maintainers who may discuss, offer constructive feedback, request changes, or approve
the work. For more information see the Code review guideline.
1. Upon receiving the sign-off of one of the maintainers you may merge your changes, or if you
   do not have permission to do that, you may request a maintainer to merge it for you.

## Attribution

This Contributing.md is adapted from Google
available at
https://github.com/google/new-project/blob/master/docs/contributing.md