/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: kd-tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.c 7057 2008-10-02 00:44:06Z gbiggs $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


#include "amcl/pf/pf_vector.h"
#include "amcl/pf/pf_kdtree.h"


// Compare keys to see if they are equal
static int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[]);

// Insert a node into the tree
static pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                               pf_kdtree_node_t *node, int key[], double value);

// Recursive node search
static pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[]);

// Recursively label nodes in this cluster
static void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth);

// Recursive node printing
//static void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node);


#ifdef INCLUDE_RTKGUI

// Recursively draw nodes
static void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig);

#endif



////////////////////////////////////////////////////////////////////////////////
// Create a tree
pf_kdtree_t *pf_kdtree_alloc(int max_size)
{
  pf_kdtree_t *self;
  //calloc 也用于分配内存空间。调用形式： (类型说明符*)calloc(n,size) 功能：在内存动态存储区中分配n块长度为“size”字节的连续区域。函数的返回值为该区域的首地址
  self = calloc(1, sizeof(pf_kdtree_t));
  //为什么是0.5？
  //每个位姿的值，都要先按比例除以此参数。如位姿是（1,2,Θ），那么在节点中的key值为（2,4，）。kdtree里保存的，是放大了的位姿
  self->size[0] = 0.50;
  self->size[1] = 0.50;
  self->size[2] = (10 * M_PI / 180);

  self->root = NULL;

  self->node_count = 0;
  self->node_max_count = max_size;
  self->nodes = calloc(self->node_max_count, sizeof(pf_kdtree_node_t));

  self->leaf_count = 0;

  return self;
}


////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
void pf_kdtree_free(pf_kdtree_t *self)
{
  free(self->nodes);
  free(self);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
void pf_kdtree_clear(pf_kdtree_t *self)
{
  self->root = NULL;
  self->leaf_count = 0;
  self->node_count = 0;

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a pose into the tree.
/*
self,一颗kdtree
pose，位姿，在节点中用key[3]表示
value，权重，如果位姿被重复插入，那么权重就会增大
*/
void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  //所有新的pose和value值，都从根节点处插入
  self->root = pf_kdtree_insert_node(self, NULL, self->root, key, value);

  // Test code
  /*
  printf("find %d %d %d\n", key[0], key[1], key[2]);
  assert(pf_kdtree_find_node(self, self->root, key) != NULL);

  pf_kdtree_print_node(self, self->root);

  printf("\n");

  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      printf("find %d %d %d\n", node->key[0], node->key[1], node->key[2]);
      assert(pf_kdtree_find_node(self, self->root, node->key) == node);
    }
  }
  printf("\n\n");
  */

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability estimate for the given pose. TODO: this
// should do a kernel density estimate rather than a simple histogram.
double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL)
    return 0.0;
  return node->value;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the cluster label for the given pose
int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}


////////////////////////////////////////////////////////////////////////////////
// Compare keys to see if they are equal
int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[])
{
  //double a, b;

  if (key_a[0] != key_b[0])
    return 0;
  if (key_a[1] != key_b[1])
    return 0;

  if (key_a[2] != key_b[2])
    return 0;

  /* TODO: make this work (pivot selection needs fixing, too)
  // Normalize angles
  a = key_a[2] * self->size[2];
  a = atan2(sin(a), cos(a)) / self->size[2];
  b = key_b[2] * self->size[2];
  b = atan2(sin(b), cos(b)) / self->size[2];

 if ((int) a != (int) b)
    return 0;
  */

  return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a node into the tree
/*
self,指定kdtree
parent，指定parent节点，主要是用来更新深度层的
node，表示从哪个节点开始插入，一般是root。因为kdtree里有一个指针nodes，就是指向根节点
key，待插入的pose
value，权重
*/
pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                        pf_kdtree_node_t *node, int key[], double value)
{
  int i;
  int split, max_split;

  // If the node doesnt exist yet...
  //如一开始，树没有节点，先创建根节点。即node=root，parent=null
  //如果指定了父节点，如node=null，parent=指定，那么创建的新的节点，就是parent的子节点
  if (node == NULL)
  {
    //不能超过最大树的范围
    assert(self->node_count < self->node_max_count);
    //根节点的地址等于树最开始的地址
    node = self->nodes + self->node_count++;
    //重新分配节点指针的内存大小
    memset(node, 0, sizeof(pf_kdtree_node_t));    

    //表示这个节点是叶子节点，可以继续插入左右节点
    node->leaf = 1;     //表示其是叶子节点了，可以往下层加两个children节点

    //也可以指定一个父节点，从父节点插入。这时候，第三个参数就不能使root节点了。因为root节点没有父节点
    if (parent == NULL)
      node->depth = 0;
    else
      node->depth = parent->depth + 1;

    for (i = 0; i < 3; i++)
      node->key[i] = key[i];

    node->value = value;
    //这是树的叶子节点总数量，根节点也是个叶子节点
    self->leaf_count += 1;      //叶子节点为啥要加1？根节点也是叶子节点？
  }

  // If the node exists, and it is a leaf node...
  else if (node->leaf)
  {
    // If the keys are equal, increment the value
    if (pf_kdtree_equal(self, key, node->key))
    {
      node->value += value;
    }

    // The keys are not equal, so split this node
    else
    {
      // Find the dimension with the largest variance and do a mean
      // split
      max_split = 0;
      node->pivot_dim = -1;
      //这里明明是选择了相差比较大的维度作为轴点的，并不是按层来的
      //因为node定义了一个povit_dim来保存选则哪个维度的
      for (i = 0; i < 3; i++)
      {
        split = abs(key[i] - node->key[i]);
        if (split > max_split)
        {
          max_split = split;
          node->pivot_dim = i;
        }
      }
      assert(node->pivot_dim >= 0);

      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;
      //将node作为父节点，讲node自身的key value与要插入的key value作为两个children传到下一层
      //children[0],表示左节点
      //为什么要把自身节点重新插入一下？    这也就出现了知乎作者提到了，如果插入n个点，树最多要2n的空间来保存
      if (key[node->pivot_dim] < node->pivot_value)
      {
        //小于pivot值的话，就将其设置为node的左子节点，同事将节点自身的值复制到其右子节点
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, key, value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
      }
      else
      {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, key, value);
      }

      node->leaf = 0;     //表示其有左右子节点了
      self->leaf_count -= 1;
    }
  }

  // If the node exists, and it has children...
  //如果不是叶子节点，即左右都有节点，那就继续往下插入
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value)
      pf_kdtree_insert_node(self, node, node->children[0], key, value);
    else
      pf_kdtree_insert_node(self, node, node->children[1], key, value);
  }

  return node;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node search
pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[])
{
  if (node->leaf)
  {
    //printf("find  : leaf %p %d %d %d\n", node, node->key[0], node->key[1], node->key[2]);

    // If the keys are the same...
    if (pf_kdtree_equal(self, key, node->key))
      return node;
    else
      return NULL;
  }
  else
  {
    //printf("find  : brch %p %d %f\n", node, node->pivot_dim, node->pivot_value);

    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    // If the keys are different...
    if (key[node->pivot_dim] < node->pivot_value)
      return pf_kdtree_find_node(self, node->children[0], key);
    else
      return pf_kdtree_find_node(self, node->children[1], key);
  }

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node printing
/*
void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node)
{
  if (node->leaf)
  {
    printf("(%+02d %+02d %+02d)\n", node->key[0], node->key[1], node->key[2]);
    printf("%*s", node->depth * 11, "");
  }
  else
  {
    printf("(%+02d %+02d %+02d) ", node->key[0], node->key[1], node->key[2]);
    pf_kdtree_print_node(self, node->children[0]);
    pf_kdtree_print_node(self, node->children[1]);
  }
  return;
}
*/


////////////////////////////////////////////////////////////////////////////////
// Cluster the leaves in the tree
void pf_kdtree_cluster(pf_kdtree_t *self)
{
  int i;
  int queue_count, cluster_count;
  pf_kdtree_node_t **queue, *node;

  queue_count = 0;
  queue = calloc(self->node_count, sizeof(queue[0]));

  // Put all the leaves in a queue
  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      node->cluster = -1;
      assert(queue_count < self->node_count);
      queue[queue_count++] = node;

      // TESTING; remove
      assert(node == pf_kdtree_find_node(self, self->root, node->key));
    }
  }

  cluster_count = 0;

  // Do connected components for each node
  while (queue_count > 0)
  {
    node = queue[--queue_count];

    // If this node has already been labelled, skip it
    if (node->cluster >= 0)
      continue;

    // Assign a label to this cluster
    node->cluster = cluster_count++;

    // Recursively label nodes in this cluster
    pf_kdtree_cluster_node(self, node, 0);
  }

  free(queue);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively label nodes in this cluster
void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth)
{
  int i;
  int nkey[3];
  pf_kdtree_node_t *nnode;

  for (i = 0; i < 3 * 3 * 3; i++)
  {
    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

    nnode = pf_kdtree_find_node(self, self->root, nkey);
    if (nnode == NULL)
      continue;

    assert(nnode->leaf);

    // This node already has a label; skip it.  The label should be
    // consistent, however.
    if (nnode->cluster >= 0)
    {
      assert(nnode->cluster == node->cluster);
      continue;
    }

    // Label this node and recurse
    nnode->cluster = node->cluster;

    pf_kdtree_cluster_node(self, nnode, depth + 1);
  }
  return;
}



#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Draw the tree
void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig)
{
  if (self->root != NULL)
    pf_kdtree_draw_node(self, self->root, fig);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively draw nodes
void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig)
{
  double ox, oy;
  char text[64];

  if (node->leaf)
  {
    ox = (node->key[0] + 0.5) * self->size[0];
    oy = (node->key[1] + 0.5) * self->size[1];

    rtk_fig_rectangle(fig, ox, oy, 0.0, self->size[0], self->size[1], 0);

    //snprintf(text, sizeof(text), "%0.3f", node->value);
    //rtk_fig_text(fig, ox, oy, 0.0, text);

    snprintf(text, sizeof(text), "%d", node->cluster);
    rtk_fig_text(fig, ox, oy, 0.0, text);
  }
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);
    pf_kdtree_draw_node(self, node->children[0], fig);
    pf_kdtree_draw_node(self, node->children[1], fig);
  }

  return;
}

#endif
