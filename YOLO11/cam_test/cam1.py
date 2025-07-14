import cv2

# 웹캠 열기 (0은 기본 웹캠을 의미)
cap = cv2.VideoCapture(1)

# 웹캠이 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("오류: 웹캠을 열 수 없습니다.")
    exit()

# 비디오 프레임을 계속해서 읽어와 화면에 표시
while True:
    # 프레임 읽기
    ret, frame = cap.read()

    # 프레임을 성공적으로 읽었는지 확인
    if not ret:
        print("오류: 프레임을 읽을 수 없습니다.")
        break

    # 읽어온 프레임을 'Webcam'이라는 이름의 창에 표시
    cv2.imshow('Webcam', frame)

    # 'q' 키를 누르면 반복문 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 사용이 끝난 후, 자원 해제
cap.release()
cv2.destroyAllWindows()
