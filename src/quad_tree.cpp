#include "quad_tree.hpp"
#include <queue>

int QuadTreeNode::maxDepth = 5;
int QuadTreeNode::maxObjPerNode = 15;
bool QuadTreeNode::is_leaf() { return children.size() == 0; }

int QuadTreeNode::num_objects() {
  reset();
  int objectCount = contents.size();
  for (int i = 0, size = contents.size(); i < size; ++i) {
    contents[i]->flag = true;
  }

  std::queue<QuadTreeNode *> process;
  process.push(this);
  while (process.size() > 0) {
    QuadTreeNode *processing = process.back();
    if (!processing->is_leaf()) {
      for (int i = 0, size = processing->children.size(); i < size; ++i) {
        process.push(&processing->children[i]);
      }
    } else {
      for (int i = 0, size = processing->contents.size(); i < size; ++i) {
        if (!processing->contents[i]->flag) {
          objectCount += 1;
          processing->contents[i]->flag = true;
        }
      }
    }
    process.pop();
  }
  reset();
  return objectCount;
}

void QuadTreeNode::insert(QuadTreeData &data) {
  if (!geom2D::rectangle_rectangle(data.bounds, nodeBounds)) {
    return;
  }
  if (is_leaf() && contents.size() + 1 > maxObjPerNode) {
    split();
  }
  if (is_leaf()) {
    contents.push_back(&data);
  } else {
    for (int i = 0, size = children.size(); i < size; ++i) {
      children[i].insert(data);
    }
  }
}

void QuadTreeNode::remove(QuadTreeData &data) {
  if (is_leaf()) {
    int removeIndex = -1;
    for (int i = 0, size = contents.size(); i < size; ++i) {
      if (contents[i]->object == data.object) {
        removeIndex = i;
        break;
      }
    }
    if (removeIndex != -1) {
      contents.erase(contents.begin() + 1);
    }
  } else {
    for (int i = 0, size = contents.size(); i < size; ++i) {
      children[i].remove(data);
    }
  }
  shake();
}

void QuadTreeNode::update(QuadTreeData &data) {
  remove(data);
  insert(data);
}

void QuadTreeNode::shake() {
  if (!is_leaf()) {
    int numObj = num_objects();
    if (num_objects == 0) {
      children.clear();
    } else if (numObj < maxObjPerNode) {
      std::queue<QuadTreeNode *> process;
      process.push(this);
      while (process.size() > 0) {
        QuadTreeNode *processing = process.back();
        if (!processing->is_leaf()) {
          for (int i = 0, size = processing->children.size(); i < size; ++i) {
            process.push(&processing->children[i]);
          }
        } else {
          contents.insert(contents.end(), processing->contents.begin(),
                          processing->contents.end());
        }
        process.pop();
      }
      children.clear();
    }
  }
}

void QuadTreeNode::split() {
  if (currentDepth + 1 >= maxDepth) {
    return;
  }
  vec2 min = geom2D::get_min(nodeBounds);
  vec2 max = geom2D::get_max(nodeBounds);
  vec2 center = min + ((max - min) * 0.5f);

  geom2D::Rectangle2D childAreas[] = {
      geom2D::Rectangle2D(
          geom2D::from_min_max(vec2{min.x, min.y}, vec2{center.x, center.y})),
      geom2D::Rectangle2D(
          geom2D::from_min_max(vec2{center.x, min.y}, vec2{max.x, center.y})),
      geom2D::Rectangle2D(
          geom2D::from_min_max(vec2{center.x, center.y}, vec2{max.x, max.y})),
      geom2D::Rectangle2D(
          geom2D::from_min_max(vec2{min.x, center.y}, vec2{center.x, max.y})),
  };

  for (int i = 0; i < 4; ++i) {
    children.push_back(QuadTreeNode(childAreas[i]));
    children[i].currentDepth = currentDepth + 1;
  }
  for (int i = 0, size = contents.size(); i < size; ++i) {
    children[i].insert(*contents[i]);
  }
  contents.clear();
}

void QuadTreeNode::reset() {
  if (is_leaf()) {
    for (int i = 0, size = contents.size(); i < size; ++i) {
      contents[i]->flag = false;
    }
  } else {
    for (int i = 0, size = children.size(); i < size; ++i) {
      children[i].reset();
    }
  }
}

std::vector<QuadTreeData *>
QuadTreeNode::query(const geom2D::Rectangle2D &area) {
  std::vector<QuadTreeData *> result;
  for (int i = 0, size = contents.size(); i < size; ++i) {
    if (!geom2D::rectangle_rectangle(contents[i]->bounds, area)) {
      result.push_back(contents[i]);
    } else {
      for (int i = 0, size = children.size(); i < size; ++i) {
        std::vector<QuadTreeData *> recurse = children[i].query(area);
        if (recurse.size() > 0) {
          result.insert(result.end(), recurse.begin(), recurse.end());
        }
      }
    }
  }
  return result;
}
