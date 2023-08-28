#ifndef _H_QUAD_TREE_
#define _H_QUAD_TREE_
#include "geometry2D.hpp"
#include <vector>

struct QuadTreeData {
  void *object;
  geom2D::Rectangle2D bounds;
  bool flag;
  inline QuadTreeData(void *o, const geom2D::Rectangle2D &b)
      : object(o), bounds(b), flag(false) {}
};

class QuadTreeNode {
protected:
  std::vector<QuadTreeNode> children;
  std::vector<QuadTreeData *> contents;
  int currentDepth;
  static int maxDepth;
  static int maxObjPerNode;
  geom2D::Rectangle2D nodeBounds;

public:
  inline QuadTreeNode(const geom2D::Rectangle2D &bounds)
      : nodeBounds(bounds), currentDepth(0) {}
  bool is_leaf();
  int num_objects();
  void insert(QuadTreeData &data);
  void remove(QuadTreeData &data);
  void update(QuadTreeData &data);
  void shake();
  void split();
  void reset();
  std::vector<QuadTreeData *> query(const geom2D::Rectangle2D &area);
};

typedef QuadTreeNode QuadTree;

#endif //_H_QUAD_TREE_