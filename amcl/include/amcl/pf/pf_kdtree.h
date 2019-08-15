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
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

#ifdef INCLUDE_RTKGUI
#include "rtk.h"
#endif


// Info for a node in the tree
typedef struct pf_kdtree_node
{
  // Depth in the tree
  //用0,1表示是否为leaf节点； depth描述的是kdtree的层
  int leaf, depth;

  // Pivot dimension and value
  int pivot_dim;          //轴点处于n维向量的第几维。
  double pivot_value;     //轴点的值，相当于split点。轴点值=（当前节点值+待加入节点值）/ 2

  // The key for this node
  //就是pose,只不过等比例放大成int
  int key[3];             

  // The value for this node
  //权重
  double value;

  // The cluster label (leaf nodes)
  //聚类的label,注意这是个label.节点label一样则为同一个了cluster
  int cluster;

  // Child nodes
  struct pf_kdtree_node *children[2];

} pf_kdtree_node_t;


// A kd tree
typedef struct
{
  // Cell size
  //位姿参数的放大倍数,为啥要放大?
  double size[3];

  // The root node of the tree
  pf_kdtree_node_t *root;

  // The number of nodes in the tree
  int node_count, node_max_count;
  //在alloc函数中，会给kdtree分配连续的节点空间。nodes指向树的开头，即根节点
  pf_kdtree_node_t *nodes;    

  // The number of leaf nodes in the tree
  int leaf_count;

} pf_kdtree_t;


// Create a tree
extern pf_kdtree_t *pf_kdtree_alloc(int max_size);

// Destroy a tree
extern void pf_kdtree_free(pf_kdtree_t *self);

// Clear all entries from the tree
extern void pf_kdtree_clear(pf_kdtree_t *self);

// Insert a pose into the tree
extern void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value);

// Cluster the leaves in the tree
extern void pf_kdtree_cluster(pf_kdtree_t *self);

// Determine the probability estimate for the given pose
//计算给定位姿在树种的概率估计,这个函数没有使用
extern double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose);

// Determine the cluster label for the given pose
//找到该位姿所在的聚类
extern int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose);


#ifdef INCLUDE_RTKGUI

// Draw the tree
extern void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig);

#endif

#endif
