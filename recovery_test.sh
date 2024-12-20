#!/bin/bash

# 반복할 횟수 설정
MAX_ITERATIONS=5

# 확인할 프로세스 이름들을 배열로 설정
PROCESS_NAMES=(
    "mint_watchdog"
    "myahrs_ros2_driver"
    "nmea_navsat_driver"
    # 필요한 프로세스 이름을 여기에 추가
)

# 색상 코드 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 카운터 초기화
counter=0

# 프로세스 종료 함수
kill_process() {
    local process_name=$1
    local pids=$(ps aux | grep "$process_name" | grep -v grep | awk '{print $2}')

    if [ -n "$pids" ]; then
        for pid in $pids; do
            echo -e "${YELLOW}프로세스 종료 시도 (PID: $pid)${NC}"
            kill $pid
        done

        # 프로세스가 종료되기를 기다림
        sleep 3

        # 종료 확인 및 강제 종료
        if ps aux | grep -v grep | grep "$process_name" > /dev/null; then
            echo -e "${RED}프로세스가 여전히 실행 중, 강제 종료 시도${NC}"
            for pid in $(ps aux | grep "$process_name" | grep -v grep | awk '{print $2}'); do
                kill -9 $pid
            done
        else
            echo -e "${GREEN}프로세스가 성공적으로 종료됨${NC}"
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
    echo "======================================================"
    echo -e "${YELLOW}반복 횟수: $(($counter + 1))/$MAX_ITERATIONS${NC}"
    echo "======================================================"

    processes_found=false

    # 각 프로세스 확인 및 종료
    for process in "${PROCESS_NAMES[@]}"
    do
        echo -e "\n${YELLOW}프로세스 확인 중: $process${NC}"

        if check_process "$process"; then
            echo -e "${GREEN}프로세스 발견: $process${NC}"
            processes_found=true
            kill_process "$process"
        else
            echo -e "${RED}프로세스가 실행중이지 않음: $process${NC}"
        fi
    done

    # 프로세스가 하나라도 발견되었다면 카운터 증가
    if [ "$processes_found" = true ]; then
        counter=$((counter + 1))

        # 다음 반복 전 대기
        echo -e "\n${YELLOW}다음 반복까지 10초 대기...${NC}"
        sleep 10
    else
        echo -e "\n${RED}모든 프로세스가 실행중이지 않습니다.${NC}"
        echo "5초 후 다시 확인합니다..."
        sleep 5
    fi
done

echo "======================================================"
echo -e "${GREEN}스크립트 완료: 총 $counter회 프로세스 종료 시도${NC}"
echo "======================================================"
