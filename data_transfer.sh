#!/bin/bash

# 입력 인자 체크
if [ "$#" -ne 1 ]; then
  echo "data_transfer.sh 패키지명"
  exit 1
fi

PACKAGE_NAME="$1"

#sshpass를 이용해 ssh 접속 시 비밀번호 자동 입력하고 rsync
rsync --rsh="sshpass -p 'everyAIRbot!4938' ssh -l airbot" -av --exclude=".git" "src/$PACKAGE_NAME" airbot@192.168.60.206:~/airbot_ws/src/
#sshpass -p 'everyAIRbot!4938' scp -r "src/$PACKAGE_NAME" airbot@192.168.60.206:~/airbot_ws/src
echo "$PACKAGE_NAME Airbot 전송 완료"
