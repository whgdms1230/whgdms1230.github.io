---
sort: 1
---

# A* Algorithms

## Heuristic 이란?

* [휴리스틱 이론 WIKI](https://ko.wikipedia.org/wiki/%ED%9C%B4%EB%A6%AC%EC%8A%A4%ED%8B%B1_%EC%9D%B4%EB%A1%A0)

휴리스틱(heuristics) 또는 발견법(發見法)이란 **불충분한 시간이나 정보로 인하여 합리적인 판단을 할 수 없거나, 체계적이면서 합리적인 판단이 굳이 필요하지 않은 상황에서 사람들이 빠르게 사용할 수 있게 보다 용이하게 구성된 간편추론의 방법**이다.

문제해결에 있어서 복잡한 문제의 경우 초기에는 휴리스틱을 이용하여 과제를 단순화시킨 후 후기에 규범적(normative)인 의사결정 규칙을 사용하고, 단순한 과업 상황에서는 처음부터 최종 의사결정에 이르기까지 규범적 규칙을 이용하여 이를 해결하려한다는 가설은 허버트 사이먼(Herbert A. Simon)이 주창한 ‘제한된 합리성(bounded rationality)’에서 시작되었고 앨런 뉴얼(Allen Newell) 등이 공동 참여하였다. ‘제한된 합리성’이란 다양한 의사결정 상황에서 인간의 인지적인 한계로 인해 발생하는 의사결정 문제를 인지적 한계 안에서 다룰 수 있는 범위로 축소시키고, **간단해진 과업의 수행에 한해 규범적 규칙을 이용한다는 것**을 의미한다.

휴리스틱의 어원은 라틴어의 ‘heuristicus’ 와 그리스어 ‘heuriskein’ 에서부터 시작되었으며, “찾아내다(find out)” 그리고 발견하다(discover)”라는 의미를 뜻한다.

## A* 알고리즘이란?

* [A* 알고리즘 WIKI](https://ko.wikipedia.org/wiki/A*_%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98)

정보과학 분야에 있어서, A* 알고리즘(A* algorithm 에이 스타 알고리즘)은 주어진 출발 꼭짓점에서부터 목표 꼭짓점까지 가는 최단 경로를 찾아내는(다시 말해 주어진 목표 꼭짓점까지 가는 최단 경로임을 판단할 수 있는 테스트를 통과하는) 그래프 탐색 알고리즘 중 하나이다. 이 알고리즘은 다익스트라 알고리즘과 유사하나 차이점은 각 꼭짓점 ***x*** 에 대해 그 꼭짓점을 통과하는 최상의 경로를 추정하는 순위값인 "휴리스틱 추정값" ***h(x)*** 을 매기는 방법을 이용한다는 것이다. 이 알고리즘은 이 휴리스틱 추정값의 순서로 꼭짓점을 방문한다. 그러므로 A* 알고리즘을 너비 우선 탐색의 한 예로 분류할 수 있다.

이 알고리즘은 1968년 피터 하트, 닐스 닐슨, 버트램 라팰이 처음 기술하였다. 그 3명의 논문에서, 이 알고리즘은 A 알고리즘(algorithm A)이라고 불렸다. 적절한 휴리스틱을 가지고 이 알고리즘을 사용하면 최적(optimal)이 된다. 그러므로 A* 알고리즘이라고 불린다.

A* 알고리즘은 출발 꼭짓점으로부터 목표 꼭짓점까지의 최적 경로를 탐색하기 위한 것이다. 이를 위해서는 각각의 꼭짓점에 대한 평가 함수를 정의해야 한다.

이는 즉, optimal한 해를 찾는 것이 아닌 방법론적(추론적)인 접근을 통해 해를 찾는 것이다.

A* 알고리즘의 핵심은 시작점부터 목적지 노드까지 갈 수 있는 모든 경로 중에 최소 비용으로 갈 수 있는 경로를 탐색하는 것으로, 이 경로의 비용을 계산해내는 것이다.

A*의 cost function은 다음과 같다.

***f(n) = g(n) + h(n)***
***g(n)*** : 출발 꼭짓점으로부터 꼭짓점 ***n*** 까지의 경로 가중치
***h(n)*** : 꼭짓점 ***n*** 으로부터 목표 꼭짓점까지의 추정 경로 가중치

일반적으로 ***g(n)*** 은 출발 꼭지점으로부터 n 꼭지점 까지의 경로 거리이며, 측정이 가능하다.
반면, ***h(n)*** 은 n 부터 도착 꼭지점 까지의 예상 cost인데, 이는 일반적으로 Manhattan Distance를 사용하며, Chebyshev Distance 또는 Euclidean Distance를 사용하기도 한다.

일반적인 A* 알고리즘은 이렇게 정의되고 사용이 되나, A* 알고리즘은 사용환경에 따라 Heuristic을 정의하는 방법에 따라 수행 결과물이 달라지기도 한다.

A* 알고리즘을 사용하고자 하는 환경에 맞게 Heuristic을 설정하면 더 좋은 결과를 만들어낼 수 있다. 단순 Manhattan Distance 만을 이용하기 보다는 주어진 환경에 맞게 Heuristic 계산 함수를 정의함으로써 실제 환경에서 경로 탐색의 효율성을 높일 수 있다.

### A* 알고리즘 예시 설명

* [A* 알고리즘 예시 설명 사이트](http://www.gisdeveloper.co.kr/?p=3897)

위 링크 작성자가 A* 알고리즘에 대하여 이해하기 쉽게 정리하였다.

### Pseudocode

```bash
pq.enqueue(start_node, g(start_node) + h(start_node))       // 우선순위 큐에 시작 노드를 삽입한다.

while pq is not empty       // 우선순위 큐가 비어있지 않은 동안
    node = pq.dequeue       // 우선순위 큐에서 pop한다.

    if node == goal_node    // 만약 해당 노드가 목표 노드이면 반복문을 빠져나온다.
        break

    for next_node in (next_node_begin...next_node_end)       // 해당 노드에서 이동할 수 있는 다음 노드들을 보는 동안
        pq.enqueue(next_node, g(node) + cost + h(next_node)) // 우선순위 큐에 다음 노드를 삽입한다.

return goal_node_dist       // 시작 노드에서 목표 노드까지의 거리를 출력한다.
```

### A* 알고리즘 C++ 코드

프로젝트를 진행하며 테스트용으로 사용한 A* 알고리즘 코드의 기본 형태로, 개선해야 할 사항은 존재하나 예시로 참조 한다.

```cpp
std::vector<uint32_t> a_star(uint32_t start, uint32_t target){

    // 시작위치와 도착위치가 같은 경우 : 경로 = 현재 위치
    if(start == target){
        std::vector<uint32_t> path;
        path.push_back(start);
        return path;
    }

    // open_list, closed_list 생성
    std::map<uint32_t, std::map<std::string, uint32_t>> open_list;
    std::map<uint32_t, std::map<std::string, uint32_t>> closed_list;

    // 시작 노드 생성, open_list에 추가
    std::map<std::string, uint32_t> root;
    // 휴리스틱은 manhattan distance로 계산
    uint32_t root_heuristic = manhattan_distance(start, target);
    root.insert(std::make_pair("loc", start));
    root.insert(std::make_pair("g_val", 0));
    root.insert(std::make_pair("h_val", root_heuristic));
    root.insert(std::make_pair("parent", 0));
    open_list.insert(std::make_pair(start, root));

    while(!open_list.empty()){
        std::map<std::string, uint32_t> cur_node;
        uint32_t min_cost = UINT32_MAX;
        // open_list 중 g_val + h_val, 즉 f 값이 가장 낮은 노드를 cur_node로 선택
        for(auto it : open_list){
            uint32_t cost = it.second.find("g_val")->second + it.second.find("h_val")->second;

            if(cost < min_cost){
                min_cost = cost;
                cur_node = it.second;
            }
        }

        // cur_node를 open_list에서 제거하고, closed_list에 등록
        uint32_t loc = cur_node.find("loc")->second;
        uint32_t g_val = cur_node.find("g_val")->second;
        uint32_t h_val = cur_node.find("h_val")->second;
        uint32_t node = loc;
        closed_list.insert(std::make_pair(node, cur_node));
        open_list.erase(node);

        // closed_list에 넣은 node가 target node와 같은 경우
        // 현재 노드부터 closed_list에서 parent 노드를 역추적 해서 경로를 생성하여 리턴
        // cal_cur_path() 는 경로 생성하는 함수
        if(node == target) return cal_cur_path(node, closed_list);

        // get_connect_nodes() 함수를 이용해 현재 노드와 연결된 노드를 탐색
        for(auto connected_node : get_connect_nodes(node)){

            // closed_list에 탐색하고자 하는 노드가 있는 경우 탐색 X
            auto close = closed_list.find(connected_node);
            if(close != closed_list.end()) continue;
            
            // 탐색하고자 하는 노드 객체 생성
            std::map<std::string, uint32_t> child_node;

            // g_val은 일정하게 1씩 증가한다고 가정
            uint32_t new_g_val = g_val + 1;
            // h_val은 manhattan distance 이용
            uint32_t new_h_val = manhattan_distance(connected_node, target);

            // open_list에 connected_node가 존재하는지 확인
            auto open = open_list.find(connected_node);
            if(open != open_list.end()){ // open_list에 connected_node가 존재하는 경우
                // open_list에 존재하는 connected_node의 f 값과
                // 현재 탐색하고 있는 connected_node의 f 값을 비교하여
                // 더 낮은 f 값을 open_list에 추가
                if(open->second.find("g_val")->second + open->second.find("h_val")->second > new_g_val + new_h_val){
                    // 현재 탐색한 connected_node의 f 값이 더 낮으므로
                    // 해당 노드를 open_list에 추가
                    child_node.insert(std::make_pair("loc", stoi(connected_node)));
                    child_node.insert(std::make_pair("g_val", new_g_val));
                    child_node.insert(std::make_pair("h_val", new_h_val));
                    child_node.insert(std::make_pair("parent", stoi(node)));

                    open_list[connected_node] = child_node;
                }
            } else { // open_list에 connected_node가 존재하지 않는 경우
                // connected_node를 open_list에 추가
                child_node.insert(std::make_pair("loc", stoi(connected_node)));
                child_node.insert(std::make_pair("g_val", new_g_val));
                child_node.insert(std::make_pair("h_val", new_h_val));
                child_node.insert(std::make_pair("parent", stoi(node)));

                open_list[connected_node] = child_node;
            }
        }
    }

    // 경로 탐색 실패
    // 현재 위치를 경로로 반환
    std::vector<uint32_t> path;
    path.push_back(start);
    return path;
}
```