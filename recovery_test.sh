#!/bin/bash

# 현재 날짜와 시간으로 로그 파일 이름 생성
LOG_DIR="./logs"
mkdir -p $LOG_DIR
LOG_FILE="$LOG_DIR/process_kill_$(date '+%Y%m%d_%H%M%S').log"

# 로그 함수 정의
log_message() {
    local message="[$(date '+%Y-%m-%d %H:%M:%S')] $1"
    echo -e "$message" | tee -a "$LOG_FILE"
}

# 색상이 있는 로그 함수 정의 (로그 파일에는 색상 코드 제외)
log_message_color() {
    local color=$1
    local message="[$(date '+%Y-%m-%d %H:%M:%S')] $2"
    echo -e "$color$message${NC}" 
    echo -e "$message" >> "$LOG_FILE"
}

# 반복할 횟수 설정
MAX_ITERATIONS=5

# 확인할 프로세스 이름들을 배열로 설정
PROCESS_NAMES=(
    "nmea_navsat_driver"
    "myahrs_ros2_driver"
    # 필요한 프로세스 이름을 여기에 추가
)

# 색상 코드 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 스크립트 시작 로그
log_message "스크립트 시작"
log_message "대상 프로세스: ${PROCESS_NAMES[*]}"
log_message "최대 반복 횟수: $MAX_ITERATIONS"
log_message "로그 파일 위치: $LOG_FILE"
log_message "----------------------------------------"

# 카운터 초기화
counter=0

# 프로세스 종료 함수
kill_process() {
    local process_name=$1
    local pids=$(ps aux | grep "$process_name" | grep -v grep | awk '{print $2}')
    
    if [ -n "$pids" ]; then
        for pid in $pids; do
            log_message_color "${YELLOW}" "프로세스 종료 시도 (프로세스: $process_name, PID: $pid)"
            kill $pid
        done
        
        # 프로세스가 종료되기를 기다림
        sleep 3
        
        # 종료 확인 및 강제 종료
        if ps aux | grep -v grep | grep "$process_name" > /dev/null; then
            log_message_color "${RED}" "프로세스가 여전히 실행 중, 강제 종료 시도 (프로세스: $process_name)"
            for pid in $(ps aux | grep "$process_name" | grep -v grep | awk '{print $2}'); do
                kill -9 $pid
                log_message_color "${RED}" "강제 종료 신호 전송 (PID: $pid)"
            done
        else
            log_message_color "${GREEN}" "프로세스가 성공적으로 종료됨 (프로세스: $process_name)"
        fi
        return 0
    fi
    return 1
}

# 프로세스 확인 함수
check_process() {
    local process_name=$1
    if ps aux | grep -v grep | grep "$process_name" > /dev/null; then
        return 0
    fi
    return 1
}

# 메인 루프
while [ $counter -lt $MAX_ITERATIONS ]
do
    log_message "======================================================"
    log_message_color "${YELLOW}" "반복 횟수: $(($counter + 1))/$MAX_ITERATIONS"
    log_message "======================================================"
    
    processes_found=false
    
    # 각 프로세스 확인 및 종료
    for process in "${PROCESS_NAMES[@]}"
    do
        log_message_color "${YELLOW}" "\n프로세스 확인 중: $process"
        
        if check_process "$process"; then
            log_message_color "${GREEN}" "프로세스 발견: $process"
            processes_found=true
            kill_process "$process"
        else
            log_message_color "${RED}" "프로세스가 실행중이지 않음: $process"
        fi
    done
    
    # 프로세스가 하나라도 발견되었다면 카운터 증가
    if [ "$processes_found" = true ]; then
        counter=$((counter + 1))
        
        # 다음 반복 전 대기
        log_message_color "${YELLOW}" "\n다음 반복까지 10초 대기..."
        sleep 10
    else
        log_message_color "${RED}" "\n모든 프로세스가 실행중이지 않습니다."
        log_message "5초 후 다시 확인합니다..."
        sleep 5
    fi
done

log_message "======================================================"
log_message_color "${GREEN}" "스크립트 완료: 총 $counter회 프로세스 종료 시도"
log_message "======================================================"

# 로그 파일 위치 출력
echo -e "\n${GREEN}로그가 다음 위치에 저장되었습니다: $LOG_FILE${NC}"
