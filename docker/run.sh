#! /bin/bash
set -e

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  --build        Build the Docker image but do not run the container."
    echo "  --run          Run the Docker container but do not build the image."
    echo "  --all          Build the Docker image and run the container."
    echo "  --help         Show this help message and exit."
}

parser() {
    for arg in "$@"; do
        case $arg in
            --build)
                BUILD=true
                ;;
            --run)
                RUN=true
                ;;
            --all)
                BUILD=true
                RUN=true
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                echo "Unknown option: $arg"
                show_help
                exit 1
                ;;
        esac
    done
}

build() {
    docker build -t touch:latest -f touch.dockerfile .
    docker build -t yarp:latest -f yarp.dockerfile .
    docker build -t ros2:latest -f ros2.dockerfile .

}

run() {
    xhost +local:root
}

main() {
    parser "$@"
    [ "$BUILD" = true ] && build
    [ "$RUN" = true ] && run
}

main "$@"