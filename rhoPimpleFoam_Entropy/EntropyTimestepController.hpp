#include <kvs/Math>
#include <kvs/Stat>
#include <kvs/Quaternion>

namespace
{

inline kvs::Quaternion log( const kvs::Quaternion& q )
{
    kvs::Quaternion lnq;

    const float norm_xyz = std::sqrt( q.x() * q.x() + q.y() * q.y() + q.z() * q.z() );
    const float theta = std::atan2( norm_xyz, q.w() );

    if(  std::abs( q.w() ) < 1 )
    {
        if( std::abs( theta ) > 0 )
        {
            lnq.x() = q.x() * theta / std::sin( theta );
            lnq.y() = q.y() * theta / std::sin( theta );
            lnq.z() = q.z() * theta / std::sin( theta );
            lnq.w() = 0.0f;

            return lnq;
        }
    }

    lnq.x() = q.x();
    lnq.y() = q.y();
    lnq.z() = q.z();
    lnq.w() = 0.0f;

    return lnq;
}

inline kvs::Quaternion exp( const kvs::Quaternion& q )
{
    kvs::Quaternion expq;

    const float theta = std::sqrt( q.x() * q.x() + q.y() * q.y() + q.z() * q.z() );
    if ( theta > 0 )
    {
        expq.x() = std::exp( q.w() ) * q.x() * std::sin( theta ) / theta;
        expq.y() = std::exp( q.w() ) * q.y() * std::sin( theta ) / theta;
        expq.z() = std::exp( q.w() ) * q.z() * std::sin( theta ) / theta;
        expq.w() = std::exp( q.w() ) * std::cos( theta );
    }
    else
    {
        expq.x() = std::exp( q.w() ) * q.x();
        expq.y() = std::exp( q.w() ) * q.y();
        expq.z() = std::exp( q.w() ) * q.z();
        expq.w() = std::exp( q.w() ) * std::cos( theta );
    }

    return expq;
}

inline kvs::Quaternion slerp( const kvs::Quaternion& q1, const kvs::Quaternion& q2, const float t , const bool shortest )
{
    kvs::Quaternion qt;

    auto qq1 = q1; qq1.normalize();
    auto qq2 = q2; qq2.normalize();
    auto dot = qq1.dot( qq2 );
    if( shortest )
    {
        if( dot < 0 )
        { 
            qq2 = -qq2;
            dot = -dot;
        }
    }

    const float phi = std::acos( dot );
    if( phi > 0 && phi < kvs::Math::pi )
    {
        qt = ( std::sin( phi * ( 1.0f - t ) ) * qq1 + std::sin( phi * t ) * qq2 ) / std::sin( phi );    
    }
    else
    {
        qt = ( 1.0f - t ) * qq1 + t * qq2;
    }
    qt.normalize();

    return qt;
}

inline kvs::Quaternion spline(
    const kvs::Quaternion& q1,
    const kvs::Quaternion& q2,
    const kvs::Quaternion& q3,
    const kvs::Quaternion& q4,
    const float t )
{
    kvs::Quaternion qt;

    auto qq1 = q1; qq1.normalize();
    auto qq2 = q2; qq2.normalize();
    auto qq3 = q3; qq3.normalize();
    auto qq4 = q4; qq4.normalize();
    
    if( qq1.dot( qq2 ) < 0 ) { qq2 = -qq2; }
    if( qq2.dot( qq3 ) < 0 ) { qq3 = -qq3; }
    if( qq3.dot( qq4 ) < 0 ) { qq4 = -qq4; }

    auto qq2i = qq2; qq2i.conjugate();
    auto qq3i = qq3; qq3i.conjugate();

    auto a2 = qq2 * exp( ( log( qq2i * qq1 ) + log( qq2i * qq3 ) ) * -0.25f );
    auto a3 = qq3 * exp( ( log( qq3i * qq2 ) + log( qq3i * qq4 ) ) * -0.25f );
    
    auto slerp1 = slerp( qq2, qq3, t, true );
    auto slerp2 = slerp( a2, a3, t, false );
    float s = 2.0f * t * ( 1.0f - t );

    qt = slerp( slerp1, slerp2, s, false );
    qt.normalize();

    return qt;
}

}

namespace local
{

inline float EntropyTimestepController::Entropy( const FrameBuffer& frame_buffer )
{
    const float p = 0.5f;
    return p * ColorEntropy( frame_buffer ) + ( 1 - p ) * DepthEntropy( frame_buffer );
    //return ColorEntropy( frame_buffer );
    //return DepthEntropy( frame_buffer );
}

inline float EntropyTimestepController::ColorEntropy( const FrameBuffer& frame_buffer )
{
    kvs::ValueArray<size_t> histogram_R( 256 );
    kvs::ValueArray<size_t> histogram_G( 256 );
    kvs::ValueArray<size_t> histogram_B( 256 );
    histogram_R.fill( 0 );
    histogram_G.fill( 0 );
    histogram_B.fill( 0 );

    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    const auto& color_buffer = frame_buffer.color_buffer;
    const auto length = depth_buffer.size();
    for ( size_t i = 0; i < length; i++ )
    {
        const auto depth = depth_buffer[i];
        const auto color_R = color_buffer[ 3 * i ];
        const auto color_G = color_buffer[ 3 * i + 1 ];
        const auto color_B = color_buffer[ 3 * i + 2 ];

        if ( depth < 1.0f )
        {
            histogram_R[ color_R ] += 1;
            histogram_G[ color_G ] += 1;
            histogram_B[ color_B ] += 1;
            n += 1;
        }
    }

    float entropy_R = 0.0f;
    float entropy_G = 0.0f;
    float entropy_B = 0.0f;
    const auto log2 = std::log( 2.0f );
    for ( const auto h : histogram_R )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_R -= p * std::log( p ) / log2; }
    }
    for ( const auto h : histogram_G )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_G -= p * std::log( p ) / log2; }
    }
    for ( const auto h : histogram_B )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_B -= p * std::log( p ) / log2; }
    }

    const auto entropy = ( entropy_R + entropy_G + entropy_B ) / 3.0f;

    return entropy;
}

inline float EntropyTimestepController::DepthEntropy( const FrameBuffer& frame_buffer )
{
    kvs::ValueArray<size_t> histogram( 256 );
    histogram.fill( 0 );

    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    for ( const auto depth : depth_buffer )
    {
        if ( depth < 1.0f )
        {
            const auto i = static_cast<int>( depth * 256 );
            histogram[i] += 1;
            n += 1;
        }
    }

    float entropy = 0.0f;
    const auto log2 = std::log( 2.0f );
    for ( const auto h : histogram )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy -= p * std::log( p ) / log2; }
    }

    return entropy;
}

inline void EntropyTimestepController::push( const Data& data )
{
    if ( m_previous_data.empty() )
    {
        // Initial step.
        this->process( data );
        m_previous_data = data;
        auto max_rotation = m_max_rotation;
        max_rotation.normalize();
        m_max_rotations.push( max_rotation );
        count++;
        m_data_queue.push( data );
    }
    else
    {
        if ( this->isCacheEnabled() )
        {
            if( m_data_queue.size() % m_interval == 0 )
            {
                this->process( data );
                auto max_rotation = m_max_rotation;
                max_rotation.normalize();
                if( max_rotation != m_max_rotations.back() && max_rotation != -m_max_rotations.back() )
                {
                    if( count == 1 )
                    {
                        std::queue<kvs::Quaternion> empty;
                        m_max_rotations.swap( empty );
                        auto start = max_rotation;
                        start.conjugate();
                        m_max_rotations.push( start );
                        count++;
                        for( size_t i = 0; i < empty.size(); i++ )
                        {
                            m_max_rotations.push( empty.front() );
                            empty.pop();
                        }
                    }
                    count++;
                }
                if( count == 4 )
                {
                    const auto q1 = m_max_rotations.front();
                    m_max_rotations.pop();
                    auto q2 = m_max_rotations.front();
                    m_max_rotations.pop();
                    auto q3 = m_max_rotations.front();
                    m_max_rotations.pop();
                    while( q2 == q3 || q2 == -q3 )
                    {
                        std::queue<kvs::Quaternion> empty;
                        m_path.swap( empty );
                        for( size_t i = 0; i < m_interval - 1; i++ )
                        {
                            m_path.push( q2 );
                        }

                        m_data_queue.pop();
                        for ( size_t i = 0; i < m_interval - 1; i++ )
                        {
                            const auto data_front = m_data_queue.front();
                            const auto rotation = m_path.front();
                            this->process( data_front, rotation );
                            m_data_queue.pop();
                            m_path.pop();
                        }
                        
                        q3 = m_max_rotations.front();
                        m_max_rotations.pop();
                    }
                    auto q4 = max_rotation;
                    
                    //this->createPathSlerp( q2, q3, m_interval );
                    this->createPathSpline( q1, q2, q3, q4, m_interval );

                    m_data_queue.pop();
                    for ( size_t i = 0; i < m_interval - 1; i++ )
                    {
                        const auto data_front = m_data_queue.front();
                        const auto rotation = m_path.front();
                        this->process( data_front, rotation );
                        m_data_queue.pop();
                        m_path.pop();
                    }
                    
                    std::queue<kvs::Quaternion> empty;
                    m_max_rotations.swap( empty );
                    m_max_rotations.push( q2 );
                    m_max_rotations.push( q3 );
                    for( size_t i = 0; i < empty.size(); i++ )
                    {
                        m_max_rotations.push( empty.front() );
                        empty.pop();
                    }
                    count--;
                }
                m_data_queue.push( data );
                m_max_rotations.push( max_rotation );
            }
            else
            {
                m_data_queue.push( data );
            }
        }
    }
}

float EntropyTimestepController::entropy( const FrameBuffer& frame_buffer )
{
    return m_entropy_function( frame_buffer );
}

inline void EntropyTimestepController::createPathSlerp(
    const kvs::Quaternion& q1,
    const kvs::Quaternion& q2,
    const size_t point_interval )
{
    std::queue<kvs::Quaternion> empty;
    m_path.swap( empty );

    for( size_t i = 1; i < point_interval; i++ )
    {
        const auto t = static_cast<double>( i ) / static_cast<double>( point_interval );
        const auto q = slerp( q1, q2, t, true );
        m_path.push( q );
    }
}

inline void EntropyTimestepController::createPathSpline(
    const kvs::Quaternion& q1,
    const kvs::Quaternion& q2,
    const kvs::Quaternion& q3,
    const kvs::Quaternion& q4,
    const size_t point_interval )
{
    std::queue<kvs::Quaternion> empty;
    m_path.swap( empty );

    for( size_t i = 1; i < point_interval; i++ )
    {
        const float t = static_cast<float>( i ) / static_cast<float>( point_interval );
        const auto q = spline( q1, q2, q3, q4, t );
        m_path.push( q );
    }
}

} // end of namespace InSituVis
