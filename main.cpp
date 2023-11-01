#include <iostream>
#include <vector>
#include <random>
#include "SStree.h"

int main() {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1000.0, 1000.0);

    std::vector<Point> points(500, Point(50));
    for (int i = 0; i < 500; i++) {
        for (int j = 0; j < 50; j++) {
            points[i][j] = dis(gen);
        }
    }

    // Insert the points into the SStree
    SsTree tree(50);
    for (auto& point : points) {
        tree.insert(point, "");
    }

    tree.test();
    tree.print();

    std::string filename = "sstree.dat";
    tree.saveToFile(filename);

    tree = SsTree(50);      // Clear the tree
    tree.loadFromFile(filename);

    tree.test();
    tree.print();

    std::vector<Point> query = tree.kNNQuery(points[0], 10);

    for (const auto& point : query) {
        std::cout << point << std::endl;
    }

    return 0;
}
