# ESP-IDF patches

Fixes to the ESP-IDF of the build image (`espressif/idf:release-v5.4`) that its release branch does not have yet.
The firmware build applies every `*.patch` here before `idf.py build`, in CI and locally:

    docker run --rm -v "$PWD":/project -w /project espressif/idf:release-v5.4 bash -c ". \$IDF_PATH/export.sh > /dev/null && git -C \$IDF_PATH apply /project/patches/esp-idf/*.patch && idf.py build"

The container is discarded after the build, the image stays as it is. A patch that no longer applies fails the
build: the release branch has moved, check whether it has the fix now and remove the patch.

## freertos-pend-yield-suspended-core.patch

From [esp-idf PR 19457](https://github.com/espressif/esp-idf/pull/19457) for
[issue 19456](https://github.com/espressif/esp-idf/issues/19456). A task woken for a core that has suspended its
scheduler, as every flash operation does, was left ready without a yield and only ran when that core rescheduled
for another reason. A timer callback calling the other core (`esp_ipc_call`) with a flash reader that never blocks
stalled the timer task, and with it the device clock, for 2 to 13 s on the bench; with the patch the clock stayed
within 10 ms.
