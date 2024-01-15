#include <bits/stdc++.h>

using namespace std;

#define ROW 9
#define COL 10

typedef pair<int, int> Pair; // (x, y)

typedef pair<double, pair<int, int>> CostPair; // (cost, (x, y))

typedef pair<double, pair<Pair, Pair>> ParentPair; // (cost, ((x, y), (px, py)))

// check whether given cell is valid or not
bool is_valid(int row, int col) {
    // return true if row num and col num is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

// check whether given cell is blocked or not
bool is_unblocked(int grid[][COL], int row, int col) {
    if (grid[row][col] == 1) {
        return true;
    } else {
        return false;
    }
}

// check whether destination cell has been reached or not
bool is_destination(int row, int col, Pair dest) {
    if (row == dest.first && col == dest.second) {
        return true;
    } else {
        return false;
    }
}

// check same position
bool is_same_pos(int row1, int col1, int row2, int col2)
{
    bool result = false;

    if (row1 == row2 && col1 == col2) result = true;

    return result;
}

// check inside open/close list
Pair is_in_list(ParentPair node, const vector<ParentPair>& open, const vector<ParentPair>& close)
{
    Pair flag_index;

    flag_index.first = 0;
    flag_index.second = -1;

    // check open list
    if (!open.empty())
    {
        for (int i = 0; i < open.size(); ++i) {
            if (is_same_pos(open[i].second.first.first, open[i].second.first.second,
                            node.second.first.first, node.second.first.second))
            {
                flag_index.first = 1;
                flag_index.second = i;
                return flag_index;
            }
        }
    }

    // check close list
    if (!close.empty())
    {
        for (int i = 0; i < close.size(); ++i) {
            if (is_same_pos(close[i].second.first.first, close[i].second.first.second,
                            node.second.first.first, node.second.first.second))
            {
                flag_index.first = 2;
                flag_index.second = i;
                return flag_index;
            }
        }
    }

    flag_index.first = 3;
    return flag_index;
}

void print_path(const vector<ParentPair>& close, Pair start)
{
    int idx = close.size() - 1;

    while (true)
    {
        ParentPair node = close[idx];

        // reached at start node
        if (is_same_pos(close[idx].second.first.first,
                        close[idx].second.first.second,
                        start.first, start.second))
        {
            cout << "(" << close[idx].second.first.first << ", "
                 << close[idx].second.first.second << ")";
            break;
        }
        else
        {
            cout << "(" << close[idx].second.first.first << ", "
                 << close[idx].second.first.second << ")" << "->";
        }

        // search parent node from close list
        for (int i = 0; i < close.size(); ++i) {
            if (is_same_pos(close[i].second.first.first,
                            close[i].second.first.second,
                            close[idx].second.second.first,
                            close[idx].second.second.second))
            {
                idx = i;
                break;
            }
        }
    }
}

void dijkstra_search(int obstacles[][COL], const vector<CostPair>& adjacent, Pair start, Pair goal)
{
    // list of calculating nodes
    vector<ParentPair> open;
    // store start node data (cost, ((x, y), (px, py)))
    open.push_back(make_pair(0.0,
                             make_pair(make_pair(start.first, start.second),
                                       make_pair(start.first, start.second))));

    // list of calculated nodes
    vector<ParentPair> close;

    // flag of found goad
    bool found_goal = false;

    // searching
    while(!open.empty())
    {
        // get node from open list
        ParentPair nop = open.front();
        open.erase(open.begin());

        // check goal found
        if (is_destination(nop.second.first.first, nop.second.first.second, goal))
        {
            cout << "Found Goal" << endl;
            found_goal = true;
            close.push_back(nop);
            break;
        }
        else
        {
            close.push_back(nop);
        }

        // adjacent 8 direction
        int adj_r, adj_c;
        double adj_cost;
        ParentPair adj_node;
        Pair flag_index;
        for (auto adj : adjacent)
        {
            adj_r = nop.second.first.first + adj.second.first; // row
            adj_c = nop.second.first.second + adj.second.second; // col
            adj_cost = nop.first + adj.first; // cost

            // adjacent node
            adj_node.first = adj_cost;
            adj_node.second.first.first = adj_r;
            adj_node.second.first.second = adj_c;
            adj_node.second.second.first = nop.second.first.first; // parent row
            adj_node.second.second.second = nop.second.first.second; // parent col

            // inside grid
            if (is_valid(adj_r, adj_c))
            {
                // not blocked by obstacle
                if (is_unblocked(obstacles, adj_r, adj_c))
                {
                    // adjacent node is in open/close list
                    flag_index = is_in_list(adj_node, open, close);

                    // in open list
                    if (flag_index.first == 1)
                    {
                        if (adj_node.first < open[flag_index.second].first)
                        {
                            open[flag_index.second].first = adj_node.first;
                            open[flag_index.second].second.second.first = nop.second.first.first;
                            open[flag_index.second].second.second.second = nop.second.first.second;
                        }
                    }
                    else if (flag_index.first == 2)
                    {
                        if (adj_node.first < close[flag_index.second].first)
                        {
                            close[flag_index.second].second.second.first = nop.second.first.first;
                            close[flag_index.second].second.second.second = nop.second.first.second;
                            open.push_back(close[flag_index.second]);
                            close.erase(close.begin() + flag_index.second);
                        }
                    }
                    else
                    {
                        open.push_back(adj_node);
                    }
                }
            }
        }
    }

    if (found_goal)
    {
        print_path(close, start);
    }
    else
    {
        cout << "Path not found" << endl;
    }
}

int main() {
    // 1 --> cell is not blocked
    // 0 --> cell is blocked
    int obstacles[ROW][COL] = {
            {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 0, 1, 1, 1, 0, 1, 1},
            {1, 1, 1, 0, 1, 1, 0, 1, 0, 1},
            {0, 0, 1, 0, 1, 0, 0, 0, 0, 1},
            {1, 1, 1, 0, 1, 1, 1, 0, 1, 0},
            {1, 0, 1, 1, 1, 1, 0, 1, 0, 0},
            {1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
            {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 0, 0, 0, 1, 0, 0, 1}
    };

    // adjacent nodes
    // pair(cost, (dx, dy))
    vector<CostPair> adjacent;
    adjacent.push_back(make_pair(1.0, make_pair(-1, 0))); // north
    adjacent.push_back(make_pair(1.0, make_pair(1, 0))); // south
    adjacent.push_back(make_pair(1.0, make_pair(0, 1))); // east
    adjacent.push_back(make_pair(1.0, make_pair(0, -1))); // west
    adjacent.push_back(make_pair(1.414, make_pair(-1, 1))); // north-east
    adjacent.push_back(make_pair(1.414, make_pair(-1, -1))); // north-west
    adjacent.push_back(make_pair(1.414, make_pair(1, 1))); // south-east
    adjacent.push_back(make_pair(1.414, make_pair(1, -1))); // south-west

    // start point
    Pair start = make_pair(8, 0);

    // goal point
    Pair goal = make_pair(0, 9);

    dijkstra_search(obstacles, adjacent, start, goal);

    return 0;
}
