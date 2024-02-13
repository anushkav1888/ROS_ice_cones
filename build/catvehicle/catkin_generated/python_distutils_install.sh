#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/anushka/ws/src/catvehicle"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/anushka/ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/anushka/ws/install/lib/python3/dist-packages:/home/anushka/ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/anushka/ws/build" \
    "/usr/bin/python3" \
    "/home/anushka/ws/src/catvehicle/setup.py" \
     \
    build --build-base "/home/anushka/ws/build/catvehicle" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/anushka/ws/install" --install-scripts="/home/anushka/ws/install/bin"
