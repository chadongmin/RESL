# Real Time Embedded System Lab in Daegu Univ.<br>
This project is about localization system made of three anchors and one tag.<br>
  * this source code is written by C.<br>
  * Hardwares that we used are DWM3000EVB & STM32F429ZI.<br>
  
--------------------------------------------------------------------
0709 커밋
* Gateway 소스코드에서 UDP Server로 message를 송신하는 함수인 udp_echoclient_send() 함수를 수정함.
* 내부 버퍼에 값을 전달하는 sprintf 함수 삭제하고 memcpy 함수 사용하여 length를 26으로 고정시킴 (null 값 만나기 전까지의 length만 버퍼에 담기는 문제 해결)
* UDP Server에 각 앵커(A1, A2, A3)와 태그 사이의 distance 값이 하나의 message로 수신되는 것 확인
