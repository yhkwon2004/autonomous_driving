import numpy as np
import cv2
from sklearn.linear_model import RANSACRegressor
import serial
import time

# model = YOLOPv2()  # YOLOPv2 모델 불러오기

ser = serial.Serial('COM10', 9600, timeout=1)
time.sleep(2)

def process_frame(frame):
    # YOLOPv2 모델을 사용하여 차선 감지
    # lane_mask = model.detect_lane(frame)

    # 차선 마크 생성
    lane_mask = np.zeros_like(frame[:, :, 0])
    lane_mask[frame.shape[0]//2:, frame.shape[1]//3:2*frame.shape[1]//3] = 1

    # 후처리 과정 1: 수평 성분 제거 및 두꺼운 차선 얇게 만들기
    kernel = np.ones((5, 5), np.uint8)
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)

    # 후처리 과정 2: Bird's-eye view 변환
    height, width = frame.shape[:2]
    src_points = np.float32([[0, height], [width, height], [width, height//2], [0, height//2]])
    dst_points = np.float32([[width//4, height], [width*3//4, height], [width*3//4, 0], [width//4, 0]])
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    lane_mask_bev = cv2.warpPerspective(lane_mask, matrix, (width, height))

    # 후처리 과정 3: ROI 설정
    roi = np.zeros_like(lane_mask_bev)
    roi[height//2:, :] = lane_mask_bev[height//2:, :]

    # 후처리 과정 4: 히스토그램을 통한 슬라이딩 윈도우
    histogram = np.sum(roi[roi.shape[0]//2:, :], axis=0)
    midpoint = np.int(histogram.shape[0] // 2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    # 슬라이딩 윈도우 설정
    n_windows = 9
    window_height = np.int(roi.shape[0] // n_windows)
    nonzero = roi.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_current = left_base
    right_current = right_base
    margin = 100
    minpix = 50

    left_lane_inds = []
    right_lane_inds = []

    for window in range(n_windows):
        win_y_low = roi.shape[0] - (window + 1) * window_height
        win_y_high = roi.shape[0] - window * window_height
        win_xleft_low = left_current - margin
        win_xleft_high = left_current + margin
        win_xright_low = right_current - margin
        win_xright_high = right_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            left_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            right_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # 2차 함수 회귀를 통한 차선 추정
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # RANSAC 알고리즘을 통한 차선 추정 (outlier 제거)
    ransac_left = RANSACRegressor()
    ransac_right = RANSACRegressor()
    ransac_left.fit(lefty.reshape(-1, 1), leftx)
    ransac_right.fit(righty.reshape(-1, 1), rightx)
    left_fit = np.polyfit(lefty, ransac_left.predict(lefty.reshape(-1, 1)), 2)
    right_fit = np.polyfit(righty, ransac_right.predict(righty.reshape(-1, 1)), 2)

    # Following line 계산
    ploty = np.linspace(0, roi.shape[0] - 1, roi.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    center_fitx = (left_fitx + right_fitx) / 2

    # 차선의 중심 좌표 계산
    center_fitx = (left_fitx + right_fitx) / 2

    # 차선 중심과 프레임 중심의 차이 계산
    frame_center = roi.shape[1] / 2
    lane_center = center_fitx[-1]
    center_offset = lane_center - frame_center

    # 차량의 이동 방향 결정 및 조향 값 계산
    steering_value = np.clip(127.5 + (center_offset * 255 / frame_center), 0, 255)
    ser.write(int(steering_value).to_bytes(1, 'big'))

    # 차선 시각화
    out_img = np.dstack((roi, roi, roi)) * 255
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    for i in range(len(ploty)):
        cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 2, (0, 255, 0), -1)
        cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 2, (0, 255, 0), -1)
        cv2.circle(out_img, (int(center_fitx[i]), int(ploty[i])), 2, (255, 255, 0), -1)

    return out_img

cap = cv2.VideoCapture('video.mp4')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame = process_frame(frame)
    cv2.imshow('Lane Detection', processed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()

