The code here is covered by Bonxai's [LICENSE](../LICENSE) file.
Please note that this is essentially a hack to fix a very specific case encountered in the ROS build farm.
I'm repeating the comment i have from [bonxai.cmake](bonxai.cmake) here.

The ros build farm uses `FETCHCONTENT_FULLY_DISCONNECTED` to perform an isolated build.
But as per the author of FetchContent himself, this is an abuse of the flag.
See https://github.com/Homebrew/brew/pull/17075 and https://gitlab.kitware.com/cmake/cmake/-/issues/25946.
Nevertheless this is a problem for us since Bonxai is not part of rosdistro (or any upstream system package repo) yet.
See here for possible progress: https://github.com/facontidavide/Bonxai/issues/55.

Since a user can use this flag as valid behavior, my best attempt at fixing this issue for the build farm is to vendor Bonxai code right here.
Hopefully soon enough, one way or another, I can remove this hack and stop vendoring bonxai code in my own repository.
