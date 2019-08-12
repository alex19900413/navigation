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
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "amcl/pf/pf.h"
#include "amcl/pf/pf_pdf.h"
#include "amcl/pf/pf_kdtree.h"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);



// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data)
{
  int i, j;
  //粒子滤波器
  pf_t *pf; 
  //粒子集合
  pf_sample_set_t *set;
  //粒子
  pf_sample_t *sample;
  
  srand48(time(NULL));

  //分配粒子空间，并初始化滤波器相关参数
  pf = calloc(1, sizeof(pf_t));
    
  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  //用户给定，默认是500,2000
  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5; 
  
  //初始化set的信息,两个sample_set
  pf->current_set = 0;
  for (j = 0; j < 2; j++)
  {
    set = pf->sets + j;
      
    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    //初始化所有粒子权重
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    //实际上这里放2倍就可以了,根据kdtree的定义
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  //set converged to 0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t *pf)
{
  int i;
  
  for (i = 0; i < 2; i++)
  {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
  
  return;
}

// Initialize the filter using a guassian
//用高斯分布的期望和方差来初始化滤波器
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;
  
  //选用curret_set这个集
  set = pf->sets + pf->current_set;
  
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  //根据提供的期望和方差,创建高斯概率密度函数。就是根据初始位姿和协方差，在初始位姿附近生成一堆粒子
  pdf = pf_pdf_gaussian_alloc(mean, cov);
    
  // Compute the new sample poses
  //根据新的位姿, 随机采样新的位姿
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    //就是根据初始位姿和协方差，在给定的初始位姿附近生成一堆粒子
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    //将随机均匀采样的粒子加入到kdtree中
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);
    
  // Re-compute cluster statistics
  pf_cluster_stats(pf, set); 

  //set converged to 0
  pf_init_converged(pf);

  return;
}


// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;  //bao

  //粒子滤波
  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  //一般设置为500个粒子.在cfg中,默认设置为5000
  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    //粒子的初始权重都一样
    sample->weight = 1.0 / pf->max_samples;
    //均匀模型,在free_space的cell上,创建一个随机的位姿
    sample->pose = (*init_fn) (init_data);

    // Add sample to histogram
    //插入kdtree
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);
  
  //set converged to 0
  pf_init_converged(pf);

  return;
}

void pf_init_converged(pf_t *pf){
  pf_sample_set_t *set;
  set = pf->sets + pf->current_set;
  set->converged = 0; 
  pf->converged = 0; 
}

int pf_update_converged(pf_t *pf)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;
  
  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold || 
       fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold){
      set->converged = 0; 
      pf->converged = 0; 
      return 0;
    }
  }
  set->converged = 1; 
  pf->converged = 1; 
  return 1; 
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;

  (*action_fn) (action_data, set);
  
  return;
}


#include <float.h>
// Update the filter with some new sensor observation
//粒子滤波器的更新。这里的data是激光雷达数据，sensor_fn是sensor观测模型，默认是似然域模型
//这个函数就是更新w_slow,w_fast，并将粒子权重均一化
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  //粒子集合
  set = pf->sets + pf->current_set;

  // Compute the sample weights
  //利用观测模型，测量粒子的总权重，并根据sensor数据，更新粒子的权重。更新完了，下面又把他们给归一化是干啥呢？归一化不等于均值化
  total = (*sensor_fn) (sensor_data, set);
  
  //似然域模型算出来的概率，什么时候会小于等于0呢？
  if (total > 0.0)
  {
    // Normalize weights
    //将各粒子集的权值归一化，不是均值化。各个粒子权值还是不一样的
    //因为有些粒子权值可能会超过1.0，重新将粒子权值设置到0-1范围
    double w_avg=0.0;
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }

    // Update running averages of likelihood of samples (Prob Rob p258)
    //更新 短期似然平均 与 长期似然平均
    //只有当w_fast < w_slow时，才会将两者置0.并均匀采样粒子。
    w_avg /= set->sample_count;
    if(pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);

    if(pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    //printf("w_avg: %e slow: %e fast: %e\n", 
           //w_avg, pf->w_slow, pf->w_fast);
  }
  else
  {
    // Handle zero total.如果等于0，则均值化处理粒子权重
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return;
}












// Resample the distribution
//低方差重采样+KLD边界，用第二个set重采样，然后将结果保存到第一个set中
void pf_update_resample(pf_t *pf)
{
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  //double r,c,U;
  //int m;
  //double count_inv;
  //保存了set_a的粒子的权重
  double* c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)

  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)//权值累积
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);
  
  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  //如果w_fast > w_slow，则什么都不做
  if(w_diff < 0.0)
    w_diff = 0.0;
  //printf("w_diff: %9.6f\n", w_diff);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  //低方差重采样，就是在已有的样本中挑选。粒子权重在前面更新过了，所以权重高的粒子会被比较大的概率随机提取
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;

    //如果w_fast > w_slow,则w_diff等于0，则会执行else
    //如果w_fast下降的w_slow更快，即有可能位置被绑架了，则会在全局地图中采用均匀采样，增加随机粒子
    //0-1之间均匀分布的随机数，按照概率增加，w_diff越大，增加粒子的可能性也越大
    if(drand48() < w_diff)
      //增加随机分布粒子，重采样
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);  
    else
    {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      //离散采样器
      double r;
      //drand48 返回服从均匀分布的·[0.0, 1.0) 之间的 double 型随机数
      r = drand48();
      //变量i不是for循环的局部变量，是函数的局部变量
      //如果跳出循环，说明两个粒子权值差别较大，
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      assert(i<set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      //在一个即生成随机数位置增加一个粒子，某个位置附近粒子数越多或者权重越大，这个位置生成重采样的粒子概率越大
      sample_b->pose = sample_a->pose;
    }

    //把采样得到的位姿的权重，设置为1
    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    //将样本添加到直方图，关于位姿的二叉树
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    //如果采样到足够多的粒子，那就退出采样。kld采样边界
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }
  
  // Reset averages, to avoid spiraling off into complete randomness.
  //避免进入完全的随机采样
  if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

  //fprintf(stderr, "\n\n");

  // Normalize weights，//重采样以后每个粒子权重=1 / M，前面把权重都设置为了1.0 了
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  
  // Re-compute cluster statistics
  //聚类，得到均值和方差等信息，将相近的一堆粒子融合成一个粒子
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set/
  //得到新粒子集。即用第一个set保存新采样的set
  pf->current_set = (pf->current_set + 1) % 2; 

  //计算滤波器是否收敛
  pf_update_converged(pf);

  free(c);
  return;
}











// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
//KLD采样的统计边界
int pf_resample_limit(pf_t *pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  //使得粒子数范围总在[min_samples, max_samples]之间。但是这个返回值有啥意义呢？
  if (n < pf->min_samples)
    return pf->min_samples;
  if (n > pf->max_samples)
    return pf->max_samples;
  
  return n;
}


// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;
  
  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);
  
  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;
  
  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;
    
    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;
        
    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
           //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    //pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}


// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  
  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;
  
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}


// Get the statistics for a particular cluster.
//获取权值最高的坐标点进行聚类，然后对高权值的聚类内的点求平均值，即当前机器人所在的位姿
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov)
{
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}


