The code here is covered by Bonxai's [LICENSE](LICENSE) file.
Please note that this is essentially a hack to fix a very specific case encountered in the ROS build farms.

The ros build farm uses `FETCHCONTENT_FULLY_DISCONNECTED` (and other flags) to perform an isolated build.
As a side note, as per the author of FetchContent himself, this is an abuse of the flag.
See https://github.com/Homebrew/brew/pull/17075 and https://gitlab.kitware.com/cmake/cmake/-/issues/25946.
Nevertheless, this is a problem for us since Bonxai is not part of rosdistro (or any upstream system package repo) yet.
See here for possible progress: https://github.com/facontidavide/Bonxai/issues/55.

I've made my best attempt at fixing this problem for isolated builds in `bonxai.cmake`. If a user runs afoul of it in an actual use case, then please raise an issue.
Hopefully soon enough, one way or another, I can remove this hack and stop vendoring bonxai code in my own repository.
