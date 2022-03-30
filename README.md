R2D2
=====================
## Задача День 1-2 [Код](https://github.com/VitalyFan/R2D2/blob/main/Day1/R2D2_day16.py)
### Созданы необходимые топики с контурами разливов и дефектов /oil_detect и /defect_detect

    oil_pub = rospy.Publisher('/oil_detect', Image) 

    defect_pub = rospy.Publisher('/defect_detect', Image)
***
### Выполнен автономный взлёт:

    navigate_wait(z=0.5, frame_id='body', auto_arm=True)

    rospy.sleep(2)
***
### Получены координаты точки взлёта/посадки (функция get_telemetry)

    telemetry = get_telemetry(frame_id='aruco_map')

    xl = telemetry.x

    yl = telemetry.y
***
### Распознан QR-код, получены и обработаны данные с него (вывод в терминал, получение координат, тип float)

    print(qr)

    qr = qr.split()

    xt = float(qr[0])

    yt = float(qr[1])
***
### Выполнен пролёт к началу трубопровода/к полученным из QR-кода координатам
    navigate_wait(x=xt, y=yt)
***
### Написана функция ждя пролёта вдеоль трубопровода и его мониторинга

    def image_line_callback(data):
    
      cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    
      img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

      yellow = cv2.inRange(img_hsv, (23,134,186), (90,180,220))

      contours_yll , _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
      contours_yll = list(contours_yll)
    
      contours_yll.sort(key=cv2.minAreaRect)

      if len(contours_yll) > 0:
    
          cnt = contours_yll[0]
        
          if cv2.contourArea(cnt) > 150:
        
              rect = cv2.minAreaRect(cnt)
            
              (x_min, y_min), (w_min, h_min), angle = rect
            
              if angle < -45:
            
                  angle = 90 + angle
                
              if w_min < h_min and angle > 0:
            
                  angle = (90 - angle) * -1
                
              if w_min > h_min and angle < 0:
            
                  angle = 90 + angle
                
              center = cv_image.shape[1] / 2
            
              error = x_min - center 

              set_velocity(vx=0.25, vy=error*(-0.01), vz=0, yaw=float('nan'), yaw_rate=angle*(0.008), frame_id='body')
