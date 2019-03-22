# 작동 시퀀스

``` mermaid
sequenceDiagram
    participant T as Task planner
    participant B as BCI

    alt eye blink가 확인되면
        B ->> T: eye blink(3)
        Note over T: 휴면상태 해제
    end

    alt 다음 경로가 1개인 노드 도달시
        Note over T: 이동목표 갱신: 다음경로와 연결된 노드

    else 다음 경로가 2개 이상인 노드 접근시
        T ->> B: cue
        B ->> T: motor imagery(좌/우)

        Note over T: 교차로 도달여부 확인
        Note over T: 입력된 방향으로 회전

        alt eye blink가 확인되면
            B ->> T: eye blink(2)
            Note over T: 이동목표 갱신: 로봇 정면에 놓인 노드
        else eye blink가 입력되지 않았다면
            Note over T: 이동목표 갱신: 마지막 경로와 연결된 노드
        end

    else 다음 경로가 없는 노드 도달시
        Note over T: 휴면상태 돌입
    end
```
