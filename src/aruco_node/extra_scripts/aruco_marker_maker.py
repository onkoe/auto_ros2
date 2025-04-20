import cv2 as cv
import numpy as np
import typer
from cv2.typing import MatLike  # something that behaves like a matrix
from typing_extensions import Annotated


def show_image(image_name: str, image: MatLike):
    # Show the resulting image
    cv.imshow("Image with ArUco Marker", image)
    cv.waitKey(0)
    cv.destroyAllWindows()


def create_perfect_camera(image_width: int, image_height: int) -> list[MatLike]:
    # Calculate image center
    cx = image_width / 2
    cy = image_height / 2

    fx = fy = 800  # focal length (x and y) in pixels

    # The camera matrix explains the intrinsics of the camera (focal length, image center, etc.)
    camera_matrix: MatLike = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
    )

    # Distant coefficients tell use how the camera distorts an image
    # like by curving the image near the edges of the image
    dist_coeffs: MatLike = np.zeros((5, 1))

    return camera_matrix, dist_coeffs


def main(
    image_width: Annotated[
        int, typer.Argument(help="The width of the image with the aruco marker")
    ],
    image_height: Annotated[
        int,
        typer.Argument(help="The height of the image with the aruco marker"),
    ],
    marker_distance: Annotated[
        float,
        typer.Argument(
            help="How far (in meters) away the aruco marker is in the image"
        ),
    ],
):
    """
    Create a white image with an aruco marker placed a specific distance away (in meters) in the image.
    """
    # Create marker rotation and translation vectors describing the marker's
    # location relative to the camera.
    # Camera Coordinate System reference: https://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf#page=4
    marker_tvec = np.array(
        [
            0.0,
            0.0,
            marker_distance,
        ],
        dtype=np.float32,
    )
    marker_rvec = np.array(
        [0.0, 0.0, 0.0], dtype=np.float32
    )  # No marker roatation!

    # Calculate where the aruco marker corners are in relation to the center of the aruco marker in 3D space
    marker_length = 0.175  # correlates to URC standard marker length in meters
    half_size = (
        marker_length / 2
    )  # distance from center of aruco marker to perimeter
    object_points: MatLike = np.array(
        [
            [-half_size, half_size, 0],  # Top-left
            [half_size, half_size, 0],  # Top-right
            [half_size, -half_size, 0],  # Bottom-right
            [-half_size, -half_size, 0],  # Bottom-left
        ],
        np.float32,
    )

    # Assume we have a perfect camera (no distortions, rotation, etc.)
    camera_matrix, dist_coeffs = create_perfect_camera(
        image_width, image_height
    )

    # Create an empty white image
    image: MatLike = np.ones((480, 640, 3), dtype=np.uint8) * 255

    # Calculate where the image points for the object points are
    # (i.e. project the 3D points of the aruco marker onto our image)
    image_points, _ = cv.projectPoints(
        object_points,
        rvec=marker_rvec,
        tvec=marker_tvec,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
    )
    image_points = np.int32(image_points).reshape(4, 2)
    top_left = image_points[3]
    bottom_right = image_points[1]

    # Resize the aruco (shrink or stretch) image based on the calcualed image points
    marker_image: MatLike = cv.imread(
        "extra_scripts/aruco_markers/aruco_marker_1.png"
    )
    side_length = bottom_right[0] - top_left[0]
    marker_image = cv.resize(
        marker_image,
        (side_length, side_length),
    )

    # Add the aruco marker to the image
    image[top_left[1] : bottom_right[1], top_left[0] : bottom_right[0]] = (
        marker_image
    )

    cv.imwrite(
        f"aruco_marker_dist_{str(marker_distance).replace('.', '_')}m.jpg",
        image,
    )


if __name__ == "__main__":
    typer.run(main)
