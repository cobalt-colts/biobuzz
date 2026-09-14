import cv2
import numpy as np
import math


# ============================================================
# CONFIG
# ============================================================

# Opponent color to avoid:
#
# Red alliance:
# AVOID_COLOR = "blue"
#
# Blue alliance:
# AVOID_COLOR = "red"
#
# Disable:
# AVOID_COLOR = None

AVOID_COLOR = "blue"


# ============================================================
# HSV THRESHOLDS
# ============================================================
#
# OpenCV HSV ranges:
#
# H = 0-179
# S = 0-255
# V = 0-255
#
# Hue has ~2 degrees per OpenCV unit.
# ============================================================

HSV_RANGES = {

    "yellow": [
        (
            (18, 218, 141),
            (32, 255, 241)
        )
    ],

    "blue": [
        (
            (95, 120, 60),
            (125, 255, 255)
        )
    ],

    # Red wraps around hue=0
    "red": [
        (
            (0, 130, 70),
            (8, 255, 255)
        ),
        (
            (170, 130, 70),
            (179, 255, 255)
        )
    ]
}


# ============================================================
# BLOB DETECTION
# ============================================================

# Image is processed at half resolution.
SCALE = 2


# Minimum yellow blob area at DOWNSCALED resolution.
#
# Lower this if small/far balls are being missed.
MIN_BLOB_AREA = 25


# Ignore gigantic detections that are probably the field,
# lighting, wall, etc.
MAX_BLOB_AREA = 50000


# This is intentionally VERY loose.
#
# We are NOT trying to prove that something is circular.
# This just rejects things like a 20:1 yellow strip.
#
# Set None to completely disable shape filtering.
MAX_ASPECT_RATIO = 4.0


# Morphology.
#
# CLOSE is especially useful because it joins broken regions
# of the same yellow ball.
KERNEL = np.ones((3, 3), np.uint8)

OPEN_ITERATIONS = 1
CLOSE_ITERATIONS = 2


# ============================================================
# CLUSTERING
# ============================================================

# Maximum gap between the bounding boxes of two yellow blobs
# before they are considered separate clusters.
#
# This is measured at FULL camera resolution.
#
# Increase = larger groups of balls become one cluster.
CLUSTER_GAP_PX = 120


# ============================================================
# CLUSTER SCORING
# ============================================================

# The biggest factor is simply:
#
#     HOW MUCH YELLOW IS THERE?
#
# This works well even when several touching balls become one
# irregular blob.

AREA_WEIGHT = 1.0


# Small bonus for having multiple separate blobs.
BLOB_COUNT_WEIGHT = 0.25


# Small preference for targets near the center of the camera.
CENTER_WEIGHT = 0.15


# ============================================================
# AVOIDANCE
# ============================================================

# Avoid-color blobs within this distance of a yellow cluster
# reduce that cluster's score.

AVOID_DISTANCE_PX = 180

AVOID_PENALTY = 1.5


# ============================================================
# MASK
# ============================================================

def build_mask(hsv, color_name):

    mask = np.zeros(
        hsv.shape[:2],
        dtype=np.uint8
    )

    for lower, upper in HSV_RANGES[color_name]:

        part = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8)
        )

        mask = cv2.bitwise_or(
            mask,
            part
        )

    # Remove isolated speckles
    if OPEN_ITERATIONS > 0:
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            KERNEL,
            iterations=OPEN_ITERATIONS
        )

    # Join broken sections of the same blob
    if CLOSE_ITERATIONS > 0:
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            KERNEL,
            iterations=CLOSE_ITERATIONS
        )

    return mask


# ============================================================
# FIND BLOBS
# ============================================================

def find_blobs(hsv, color_name, scale):

    mask = build_mask(
        hsv,
        color_name
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    blobs = []

    for contour in contours:

        area_small = cv2.contourArea(
            contour
        )

        if area_small < MIN_BLOB_AREA:
            continue

        if area_small > MAX_BLOB_AREA:
            continue


        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        x, y, w, h = cv2.boundingRect(
            contour
        )

        if w <= 0 or h <= 0:
            continue


        # ----------------------------------------------------
        # Extremely loose shape rejection
        # ----------------------------------------------------

        if MAX_ASPECT_RATIO is not None:

            aspect = max(w, h) / float(
                max(1, min(w, h))
            )

            if aspect > MAX_ASPECT_RATIO:
                continue


        # ----------------------------------------------------
        # Centroid
        # ----------------------------------------------------

        moments = cv2.moments(
            contour
        )

        if moments["m00"] != 0:

            cx_small = (
                moments["m10"]
                / moments["m00"]
            )

            cy_small = (
                moments["m01"]
                / moments["m00"]
            )

        else:

            cx_small = x + w / 2
            cy_small = y + h / 2


        # ----------------------------------------------------
        # Scale everything back to full camera resolution
        # ----------------------------------------------------

        contour_full = (
            contour.astype(np.float32)
            * scale
        ).astype(np.int32)


        blob = {

            "contour": contour_full,

            "area":
                area_small
                * scale
                * scale,

            "cx":
                int(cx_small * scale),

            "cy":
                int(cy_small * scale),

            "x":
                int(x * scale),

            "y":
                int(y * scale),

            "w":
                int(w * scale),

            "h":
                int(h * scale),
        }

        blobs.append(
            blob
        )

    return blobs, mask


# ============================================================
# RECTANGLE GAP
# ============================================================

def blob_gap(a, b):

    a_left = a["x"]
    a_right = a["x"] + a["w"]

    a_top = a["y"]
    a_bottom = a["y"] + a["h"]


    b_left = b["x"]
    b_right = b["x"] + b["w"]

    b_top = b["y"]
    b_bottom = b["y"] + b["h"]


    # Horizontal gap

    dx = max(
        b_left - a_right,
        a_left - b_right,
        0
    )


    # Vertical gap

    dy = max(
        b_top - a_bottom,
        a_top - b_bottom,
        0
    )


    return math.sqrt(
        dx * dx
        + dy * dy
    )


# ============================================================
# CLUSTER CONNECTION
# ============================================================

def blobs_connected(a, b):

    return (
        blob_gap(a, b)
        <= CLUSTER_GAP_PX
    )


# ============================================================
# BUILD CLUSTERS
# ============================================================

def build_clusters(blobs):

    clusters = []

    visited = [
        False
    ] * len(blobs)


    for start in range(
        len(blobs)
    ):

        if visited[start]:
            continue


        queue = [start]

        visited[start] = True

        cluster = []


        while queue:

            current_index = queue.pop()

            current = blobs[
                current_index
            ]

            cluster.append(
                current
            )


            for i in range(
                len(blobs)
            ):

                if visited[i]:
                    continue


                other = blobs[i]


                if blobs_connected(
                    current,
                    other
                ):

                    visited[i] = True

                    queue.append(
                        i
                    )


        clusters.append(
            cluster
        )


    return clusters


# ============================================================
# POINT / RECTANGLE DISTANCE
# ============================================================

def point_to_blob_distance(px, py, blob):

    left = blob["x"]
    right = blob["x"] + blob["w"]

    top = blob["y"]
    bottom = blob["y"] + blob["h"]


    dx = max(
        left - px,
        px - right,
        0
    )

    dy = max(
        top - py,
        py - bottom,
        0
    )


    return math.sqrt(
        dx * dx
        + dy * dy
    )


# ============================================================
# ANALYZE CLUSTER
# ============================================================

def analyze_cluster(
    cluster,
    avoided,
    image_width,
    image_height
):

    # --------------------------------------------------------
    # Total yellow area
    # --------------------------------------------------------

    total_area = sum(
        blob["area"]
        for blob in cluster
    )


    # --------------------------------------------------------
    # Area-weighted center
    # --------------------------------------------------------

    cx = int(
        sum(
            blob["cx"]
            * blob["area"]
            for blob in cluster
        )
        / max(1, total_area)
    )

    cy = int(
        sum(
            blob["cy"]
            * blob["area"]
            for blob in cluster
        )
        / max(1, total_area)
    )


    # --------------------------------------------------------
    # Bounding box around entire cluster
    # --------------------------------------------------------

    left = min(
        blob["x"]
        for blob in cluster
    )

    top = min(
        blob["y"]
        for blob in cluster
    )

    right = max(
        blob["x"] + blob["w"]
        for blob in cluster
    )

    bottom = max(
        blob["y"] + blob["h"]
        for blob in cluster
    )


    width = right - left
    height = bottom - top


    # Keep roughly the same llpython concept as before.
    #
    # This is NOT a fitted circle.
    # It is just half the cluster bounding-box diagonal.

    radius = int(
        math.sqrt(
            width * width
            + height * height
        )
        / 2
    )


    # --------------------------------------------------------
    # Entry blob
    # --------------------------------------------------------
    #
    # Largest yellow blob is generally the most obvious /
    # closest acquisition point.
    # --------------------------------------------------------

    entry = max(
        cluster,
        key=lambda blob:
            blob["area"]
    )


    # --------------------------------------------------------
    # Distance from image center
    # --------------------------------------------------------

    image_cx = image_width / 2.0
    image_cy = image_height / 2.0


    center_distance = math.sqrt(
        (cx - image_cx) ** 2
        +
        (cy - image_cy) ** 2
    )


    max_center_distance = math.sqrt(
        image_cx ** 2
        +
        image_cy ** 2
    )


    normalized_center_distance = (
        center_distance
        /
        max(
            1,
            max_center_distance
        )
    )


    # --------------------------------------------------------
    # Avoidance
    # --------------------------------------------------------

    danger = 0.0
    nearby_avoided = 0


    for bad in avoided:

        distance = point_to_blob_distance(
            bad["cx"],
            bad["cy"],
            {
                "x": left,
                "y": top,
                "w": width,
                "h": height
            }
        )


        if distance < AVOID_DISTANCE_PX:

            nearby_avoided += 1

            danger += (
                1.0
                -
                distance
                / AVOID_DISTANCE_PX
            )


    # --------------------------------------------------------
    # SCORE
    # --------------------------------------------------------
    #
    # Main goal:
    #
    #     find the biggest concentration of yellow.
    #
    # sqrt(area) prevents one huge blob from completely
    # dominating everything.
    # --------------------------------------------------------

    area_score = math.sqrt(
        total_area
    )


    score = (
        area_score
        * AREA_WEIGHT
    )

    score += (
        len(cluster)
        * BLOB_COUNT_WEIGHT
        * 10.0
    )

    score -= (
        normalized_center_distance
        * CENTER_WEIGHT
        * 10.0
    )

    score -= (
        danger
        * AVOID_PENALTY
        * 10.0
    )


    return {

        "blobs": cluster,

        "count":
            len(cluster),

        "area":
            total_area,

        "cx":
            cx,

        "cy":
            cy,

        "x":
            left,

        "y":
            top,

        "w":
            width,

        "h":
            height,

        "radius":
            radius,

        "entry":
            entry,

        "nearby_avoided":
            nearby_avoided,

        "danger":
            danger,

        "score":
            score
    }


# ============================================================
# DRAW CROSSHAIR
# ============================================================

def draw_crosshair(
    image,
    x,
    y,
    color
):

    cv2.line(
        image,
        (x - 12, y),
        (x + 12, y),
        color,
        2
    )

    cv2.line(
        image,
        (x, y - 12),
        (x, y + 12),
        color,
        2
    )


# ============================================================
# LIMELIGHT PIPELINE
# ============================================================

def runPipeline(image, llrobot):

    largestContour = np.array(
        [[]]
    )


    # --------------------------------------------------------
    # llpython
    #
    # 0 = total yellow blobs
    # 1 = blobs in selected cluster
    # 2 = cluster center X
    # 3 = cluster center Y
    # 4 = cluster approximate radius
    # 5 = entry blob X
    # 6 = entry blob Y
    # 7 = nearby avoided-color blobs
    # --------------------------------------------------------

    llpython = [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    ]


    if (
        image is None
        or image.size == 0
    ):

        return (
            largestContour,
            image,
            llpython
        )


    h_img, w_img = image.shape[:2]


    # ========================================================
    # DOWNSCALE
    # ========================================================

    scale = SCALE

    if (
        w_img // scale
        < 40
    ):
        scale = 1


    if scale != 1:

        small = cv2.resize(
            image,
            (
                w_img // scale,
                h_img // scale
            ),
            interpolation=cv2.INTER_AREA
        )

    else:

        small = image


    # ========================================================
    # HSV
    # ========================================================

    hsv = cv2.cvtColor(
        small,
        cv2.COLOR_BGR2HSV
    )


    # ========================================================
    # YELLOW BLOBS
    # ========================================================

    yellow_blobs, yellow_mask = find_blobs(
        hsv,
        "yellow",
        scale
    )


    # ========================================================
    # AVOIDED COLOR BLOBS
    # ========================================================

    avoided = []


    if AVOID_COLOR is not None:

        avoided, _ = find_blobs(
            hsv,
            AVOID_COLOR,
            scale
        )


    # ========================================================
    # CLUSTERS
    # ========================================================

    raw_clusters = build_clusters(
        yellow_blobs
    )


    clusters = []


    for raw_cluster in raw_clusters:

        cluster = analyze_cluster(
            raw_cluster,
            avoided,
            w_img,
            h_img
        )

        clusters.append(
            cluster
        )


    clusters.sort(
        key=lambda cluster:
            cluster["score"],
        reverse=True
    )


    # ========================================================
    # DRAW YELLOW BLOBS
    # ========================================================

    for blob in yellow_blobs:

        # Draw actual detected contour
        cv2.drawContours(
            image,
            [blob["contour"]],
            -1,
            (0, 255, 255),
            2
        )


        # Bounding box
        cv2.rectangle(
            image,
            (
                blob["x"],
                blob["y"]
            ),
            (
                blob["x"] + blob["w"],
                blob["y"] + blob["h"]
            ),
            (0, 255, 255),
            1
        )


    # ========================================================
    # DRAW AVOIDED BLOBS
    # ========================================================

    for blob in avoided:

        cv2.drawContours(
            image,
            [blob["contour"]],
            -1,
            (0, 0, 255),
            2
        )


        cv2.line(
            image,
            (
                blob["x"],
                blob["y"]
            ),
            (
                blob["x"] + blob["w"],
                blob["y"] + blob["h"]
            ),
            (255, 255, 255),
            2
        )


        cv2.line(
            image,
            (
                blob["x"] + blob["w"],
                blob["y"]
            ),
            (
                blob["x"],
                blob["y"] + blob["h"]
            ),
            (255, 255, 255),
            2
        )


    # ========================================================
    # DRAW ALL CLUSTERS
    # ========================================================

    for i, cluster in enumerate(
        clusters
    ):

        cv2.rectangle(
            image,
            (
                cluster["x"],
                cluster["y"]
            ),
            (
                cluster["x"]
                + cluster["w"],

                cluster["y"]
                + cluster["h"]
            ),
            (180, 180, 180),
            1
        )


        text = (
            "C%d blobs=%d area=%d score=%.1f"
            % (
                i,
                cluster["count"],
                cluster["area"],
                cluster["score"]
            )
        )


        cv2.putText(
            image,
            text,
            (
                cluster["x"],
                max(
                    15,
                    cluster["y"] - 5
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )


    # ========================================================
    # SELECT BEST CLUSTER
    # ========================================================

    if len(clusters) > 0:

        target = clusters[0]

        entry = target["entry"]


        # ----------------------------------------------------
        # Green box = target cluster
        # ----------------------------------------------------

        cv2.rectangle(
            image,
            (
                target["x"] - 3,
                target["y"] - 3
            ),
            (
                target["x"]
                + target["w"]
                + 3,

                target["y"]
                + target["h"]
                + 3
            ),
            (0, 255, 0),
            3
        )


        draw_crosshair(
            image,
            target["cx"],
            target["cy"],
            (0, 255, 0)
        )


        # ----------------------------------------------------
        # White box = suggested entry blob
        # ----------------------------------------------------

        cv2.rectangle(
            image,
            (
                entry["x"] - 3,
                entry["y"] - 3
            ),
            (
                entry["x"]
                + entry["w"]
                + 3,

                entry["y"]
                + entry["h"]
                + 3
            ),
            (255, 255, 255),
            2
        )


        cv2.line(
            image,
            (
                entry["cx"],
                entry["cy"]
            ),
            (
                target["cx"],
                target["cy"]
            ),
            (255, 255, 255),
            1
        )


        largestContour = entry[
            "contour"
        ]


        llpython = [

            # total visible yellow blobs
            len(
                yellow_blobs
            ),

            # blobs in target cluster
            target[
                "count"
            ],

            # target X
            target[
                "cx"
            ],

            # target Y
            target[
                "cy"
            ],

            # approximate target radius
            target[
                "radius"
            ],

            # entry X
            entry[
                "cx"
            ],

            # entry Y
            entry[
                "cy"
            ],

            # nearby opponent blobs
            target[
                "nearby_avoided"
            ]
        ]


    # ========================================================
    # STATUS
    # ========================================================

    avoid_text = (
        AVOID_COLOR.upper()
        if AVOID_COLOR is not None
        else "NONE"
    )


    cv2.putText(
        image,
        "YELLOW BLOBS:%d  CLUSTERS:%d  AVOID:%s"
        % (
            len(yellow_blobs),
            len(clusters),
            avoid_text
        ),
        (
            5,
            h_img - 8
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 255, 0),
        1,
        cv2.LINE_AA
    )


    return (
        largestContour,
        image,
        llpython
    )