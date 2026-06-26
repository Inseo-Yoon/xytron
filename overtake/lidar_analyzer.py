import math

###############################################################################
# LidarAnalyzer
# 라이다 ranges 배열에서 앞차/옆차 거리를 분석합니다.
#
# [실측 기반 인덱스 구조 - 360개, 1도 간격]
# ranges[0] = 정면, 반시계 방향으로 증가
#
#   정면 (앞차):  index   0 ~  30  (정면 ±30도)
#   좌측 (옆차):  index  60 ~  98  (90도 방향)
#   차체 노이즈:  index  99 ~ 261
#   우측 (옆차):  index 262 ~ 310  (270도 방향)
###############################################################################

# 수정 예시
NOISE_MAX_DIST    = 0.1  # 더 엄격하게 노이즈 제거
FRONT_TRIGGER_DIST = 6.0 # 그대로 유지
LEFT_DETECT_DIST  = 5.0  # 그대로 유지
RIGHT_DETECT_DIST = 5.0  # 그대로 유지
MIN_VALID_DIST    = 0.1  # 센서가 유효하게 읽는 하한선

FRONT_SLICE = slice(0, 15)
LEFT_SLICE  = slice(60, 98)
RIGHT_SLICE = slice(262, 310)


def find_noise_boundaries(ranges):
    ranges = list(ranges)

    best_start = -1
    best_end   = -1
    best_len   = 0
    cur_start  = -1
    cur_len    = 0

    for i, r in enumerate(ranges):
        if not math.isinf(r) and r <= NOISE_MAX_DIST:
            if cur_start == -1:
                cur_start = i
            cur_len += 1
        else:
            if cur_len > best_len:
                best_len   = cur_len
                best_start = cur_start
                best_end   = cur_start + cur_len - 1
            cur_start = -1
            cur_len   = 0

    if cur_len > best_len:
        best_start = cur_start
        best_end   = cur_start + cur_len - 1

    return best_start, best_end


def get_front_distance(ranges):
    if ranges is None or len(ranges) == 0:
        return float('inf')
    vals = [r for r in ranges[FRONT_SLICE] if not math.isinf(r) and r > MIN_VALID_DIST]
    return float(min(vals)) if vals else float('inf')


def get_left_distance(ranges):
    if ranges is None or len(ranges) == 0:
        return float('inf')
    vals = [r for r in ranges[LEFT_SLICE] if not math.isinf(r) and r > MIN_VALID_DIST]
    return float(min(vals)) if vals else float('inf')


def get_right_distance(ranges):
    if ranges is None or len(ranges) == 0:
        return float('inf')
    vals = [r for r in ranges[RIGHT_SLICE] if not math.isinf(r) and r > MIN_VALID_DIST]
    return float(min(vals)) if vals else float('inf')


def is_front_car_detected(ranges):
    return get_front_distance(ranges) < FRONT_TRIGGER_DIST


def is_left_car_detected(ranges):
    return get_left_distance(ranges) < LEFT_DETECT_DIST


def is_right_car_detected(ranges):
    return get_right_distance(ranges) < RIGHT_DETECT_DIST
