import cv2
import numpy as np

def create_marker_with_white_border():
    # 1. 딕셔너리 설정 (DICT_4X4_50)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # 2. 마커 생성 (검은색 테두리 포함, 내부 300픽셀)
    # 구버전 OpenCV 호환을 위해 drawMarker 사용
    marker_img = cv2.aruco.drawMarker(aruco_dict, 5, 300, borderBits=1)

    # 3. 흰색 배경(Padding) 추가 
    # 사방으로 50픽셀씩 흰색(255)을 추가합니다.
    # 결과 이미지 크기: 300 + 50 + 50 = 400x400 픽셀
    border_size = 50
    final_img = cv2.copyMakeBorder(
        marker_img,
        top=border_size,
        bottom=border_size,
        left=border_size,
        right=border_size,
        borderType=cv2.BORDER_CONSTANT,
        value=255 # 255는 흰색 (Grayscale 기준)
    )

    # 4. 파일 저장
    filename = "marker_id5.png"
    cv2.imwrite(filename, final_img)
    
    print(f"✅ 생성 완료: {filename}")
    print(f"   - 전체 크기: {final_img.shape}")
    print("   - 특징: 마커 주변에 흰색 여백이 추가되어 인식이 훨씬 잘 됩니다.")

if __name__ == "__main__":
    create_marker_with_white_border()