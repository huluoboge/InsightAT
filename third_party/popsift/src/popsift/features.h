/*
 * Copyright 2017, Simula Research Laboratory
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "sift_constants.h"

#include <iostream>
#include <vector>

namespace popsift {

struct Descriptor; // float features[128];

/**
 * @brief This is a data structure that is returned to a calling program.
 * The xpos/ypos information in feature is scale-adapted.
 */
struct Feature
{
    int         debug_octave;
    float       xpos;
    float       ypos;
    /// scale
    float       sigma;
    /// number of this extremum's orientations
    /// remaining entries in desc are 0
    int         num_ori;
    float       orientation[ORIENTATION_MAX_COUNT];
    Descriptor* desc[ORIENTATION_MAX_COUNT];

    void print( std::ostream& ostr, bool write_as_uchar ) const;
};

std::ostream& operator<<( std::ostream& ostr, const Feature& feature );

class FeaturesBase
{
    int          _num_ext;
    int          _num_ori;

public:
    FeaturesBase( );
    virtual~ FeaturesBase( );

    inline int     size() const                { return _num_ext; }
    inline int     getFeatureCount() const     { return _num_ext; }
    inline int     getDescriptorCount() const  { return _num_ori; }

    inline void    setFeatureCount( int num_ext )    { _num_ext = num_ext; }
    inline void    setDescriptorCount( int num_ori ) { _num_ori = num_ori; }
};

/**
 * @brief This is a data structure that is returned to a calling program.
 * _ori is a transparent flat memory holding descriptors
 * that are referenced by the extrema.
 *
 * Note that the current data structures do not allow to match
 * Descriptors in the transparent array with their extrema except
 * for brute force.
 *
 * Note: FeaturesHost is typedef'd to its older name Features
 */
class FeaturesHost : public FeaturesBase
{
    Feature*     _ext;
    Descriptor*  _ori;

public:
    FeaturesHost( );
    FeaturesHost( int num_ext, int num_ori );
    ~FeaturesHost( ) override;

    typedef Feature*       F_iterator;
    typedef const Feature* F_const_iterator;

    inline F_iterator       begin()       { return _ext; }
    inline F_const_iterator begin() const { return _ext; }
    inline F_iterator       end()         { return &_ext[size()]; }
    inline F_const_iterator end() const   { return &_ext[size()]; }

    void reset( int num_ext, int num_ori );
    void pin( );
    void unpin( );

    inline Feature*    getFeatures()    { return _ext; }
    inline Descriptor* getDescriptors() { return _ori; }

    void print( std::ostream& ostr, bool write_as_uchar ) const;

protected:
    friend class Pyramid;
};

using Features = FeaturesHost;

std::ostream& operator<<( std::ostream& ostr, const FeaturesHost& feature );

class FeaturesDev : public FeaturesBase
{
    Feature*     _ext;
    Descriptor*  _ori;
    int*         _rev; // the reverse map from descriptors to extrema

public:
    FeaturesDev( );
    FeaturesDev( int num_ext, int num_ori );
    ~FeaturesDev( ) override;

    void reset( int num_ext, int num_ori );

    void match( FeaturesDev* other );

    inline Feature*    getFeatures()    { return _ext; }
    inline Descriptor* getDescriptors() { return _ori; }
    inline int*        getReverseMap()  { return _rev; }
};

/**
 * @brief One row of brute-force SIFT descriptor matching (same CUDA kernel strategy as FeaturesDev::match).
 */
struct DescriptorBfMatchRow
{
    int   best_train_idx;
    int   second_train_idx;
    int   accepted;       ///< 1 if the match passes the configured ratio test (or ratio test disabled)
    float best_dist_sq;   ///< squared L2 distance between query descriptor and best_train_idx
};

/**
 * @brief Host-callable brute-force descriptor matching (PopSIFT CUDA kernel).
 *
 * Uploads @p query_count descriptors from @p query_desc and @p train_count from @p train_desc,
 * runs the same nearest / second-nearest search as the internal @c compute_distance kernel,
 * and returns one row per query descriptor (size == query_count).
 *
 * @param query_desc row-major float array, @p query_count * 128 values
 * @param train_desc row-major float array, @p train_count * 128 values
 */
std::vector<DescriptorBfMatchRow> match_descriptors_bruteforce(
    const float* query_desc, int query_count,
    const float* train_desc, int train_count,
    float ratio_threshold,
    bool use_ratio_test );

} // namespace popsift
