#include <kvs/Math>
#include <kvs/Stat>
#include <kvs/Quaternion>

namespace
{

inline kvs::Quaternion slerp( const kvs::Quaternion& q1, const kvs::Quaternion& q2, const float t )
{
    auto qq1 = q1; qq1.normalize();
    auto qq2 = q2; qq2.normalize();
    auto dot = qq1.dot( qq2 );
    if( dot < 0 )
    { 
        qq2 = -qq2;
        dot = -dot;
    }

    auto phi = std::acos( dot );
    auto qt = ( std::sin( phi * ( 1 - t ) ) * qq1 + std::sin( phi * t ) * qq2 ) / std::sin( phi );
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
    auto qq1 = q1; qq1.normalize();
    auto qq2 = q2; qq2.normalize();
    if( qq1.dot( qq2 ) < 0 ) { qq2 = -qq2; }
    auto qq3 = q3; qq3.normalize();
    if( qq2.dot( qq3 ) < 0 ) { qq3 = -qq3; }
    auto qq4 = q4; qq4.normalize();
    if( qq3.dot( qq4 ) < 0 ) { qq4 = -qq4; }

    auto qq2i = qq2; qq2i.conjugate();
    auto qq3i = qq3; qq3i.conjugate();

    const auto a2 = qq2 * ( ( ( qq2i * qq1 ).log() + ( qq2i * qq3 ).log() ) / -4.0f ).exp();
    const auto a3 = qq3 * ( ( ( qq3i * qq2 ).log() + ( qq3i * qq4 ).log() ) / -4.0f ).exp();
    
    auto qt = slerp( slerp( q2, q3, t ), slerp( a2, a3, t ), 2.0f * t * ( 1 - t ) );
    qt.normalize();

    return qt;
}

inline kvs::Vec3 calcUpVector( const kvs::Vec3& xyz )
{
    const float x = xyz[0];
    const float y = xyz[1];
    const float z = xyz[2];
    kvs::Vec3 u;

    if( ( x == 0 ) && ( z == 0 ) ){
        u = kvs::Vec3( { 0.0f, 0.0f, -1.0f } );
    }
    else
    {
        float r, t, p;
        r = sqrt( xyz.dot( xyz ) );
        t = acos( y / r );
        if ( x >= 0 )
        {
            p = acos( z / sqrt( z * z + x * x ) );
        }
        else
        {
            p = -1 * acos( z / sqrt( z * z + x * x ) );
        }
        if ( p < 0 )
        {
            p += 2 * kvs::Math::pi;
        }
                        
        auto rtp = kvs::Vec3( { r, t, p } );
                        
        kvs::Vec3 pp;
        if ( rtp[1] > kvs::Math::pi / 2 )
        {
            pp = rtp - kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        else
        {
            pp = rtp + kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        const float pp_x = pp[0] * std::sin( pp[1] ) * std::sin( pp[2] );
        const float pp_y = pp[0] * std::cos( pp[1] );
        const float pp_z = pp[0] * std::sin( pp[1] ) * std::cos( pp[2] );
        if ( rtp[1] > kvs::Math::pi / 2 )
        {
            u = kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
        else
        {
            u = -1 * kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
    }

    return u;
}

}

namespace local
{

inline float EntropyTimestepController::Entropy( const FrameBuffer& frame_buffer )
{
    const float p = 0.5f;
    //return p * ColorEntropy( frame_buffer ) + ( 1 - p ) * DepthEntropy( frame_buffer );
    return ColorEntropy( frame_buffer );
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
        auto max_position_q = kvs::Quaternion::RotationQuaternion( m_position0, m_max_position );
        max_position_q.normalize();
        auto start_position = kvs::Vec3( { 0.0f, 0.0f, 12.0f } );
        if( start_position == m_max_position )
        {
            start_position = kvs::Vec3( { 0.0f, 0.0f, -12.0f } );
        }
        auto start_position_q = kvs::Quaternion::RotationQuaternion( m_position0, start_position );
        start_position_q.normalize();
        m_max_positions.push( start_position_q );
        count += 1;
        m_max_positions.push( max_position_q );
        count += 1;
        m_data_queue.push( data );
    }
    else
    {
        if ( this->isCacheEnabled() )
        {
            if( m_data_queue.size() % m_interval == 0 )
            {
                this->process( data );
                auto max_position_q = kvs::Quaternion::RotationQuaternion( m_position0, m_max_position );
                max_position_q.normalize();
                if( max_position_q != m_max_positions.back() )
                {
                    count += 1;
                }
                if( count == 4 )
                {
                    const auto q1 = m_max_positions.front();
                    m_max_positions.pop();
                    auto q2 = m_max_positions.front();
                    m_max_positions.pop();
                    auto q3 = m_max_positions.front();
                    m_max_positions.pop();
                    while( q2 == q3 )
                    {
                        const kvs::Vec3 l = { 0.0f, 0.0f, 0.0f };
                        const auto d = InSituVis::Viewpoint::Direction::Uni;
                        auto path = InSituVis::Viewpoint();
                        const auto p = kvs::Quaternion::Rotate( m_position0, q2 );
                        const auto u = calcUpVector( p );
                        for( size_t i = 0; i < m_interval - 1; i++ )
                        {
                            path.add( { d, p, u, l } );
                        }
                        m_path = path;

                        m_data_queue.pop();
                        for ( size_t i = 0; i < m_interval - 1; i++ )
                        {
                            const auto data_front = m_data_queue.front();
                            this->process( data_front, i );
                            m_data_queue.pop();
                        }
                        m_path.clear();
                        
                        q3 = m_max_positions.front();
                        m_max_positions.pop();
                    }
                    auto q4 = max_position_q;
                    
                    if( q1.dot( q2 ) < 0 ) { q2 = -q2; }
                    if( q2.dot( q3 ) < 0 ) { q3 = -q3; }
                    if( q3.dot( q4 ) < 0 ) { q4 = -q4; }
                    m_path = this->createPathSpline( q1, q2, q3, q4, m_interval );
                    m_data_queue.pop();
                    for ( size_t i = 0; i < m_interval - 1; i++ )
                    {
                        const auto data_front = m_data_queue.front();
                        this->process( data_front, i );
                        m_data_queue.pop();
                    }
                    m_path.clear();
                    
                    std::queue<kvs::Quaternion> position_queue;
                    position_queue.swap( m_max_positions );
                    m_max_positions.push( q2 );
                    m_max_positions.push( q3 );
                    for( size_t i = 0; i < position_queue.size(); i++ )
                    {
                        m_max_positions.push( position_queue.front() );
                        position_queue.pop();
                    }
                    count -= 1;
                }
                m_data_queue.push( data );
                m_max_positions.push( max_position_q );
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

inline InSituVis::Viewpoint EntropyTimestepController::createPathSpline(
    const kvs::Quaternion& q1,
    const kvs::Quaternion& q2,
    const kvs::Quaternion& q3,
    const kvs::Quaternion& q4,
    const size_t point_interval )
{
    using Viewpoint = InSituVis::Viewpoint;
    const kvs::Vec3 l = { 0.0f, 0.0f, 0.0f };
    const auto d = Viewpoint::Direction::Uni;
    auto path = Viewpoint();

    for( size_t i = 1; i < point_interval; i++ )
    {
        const auto t = static_cast<double>( i ) / static_cast<double>( point_interval );
        //const auto q = kvs::Quaternion::SplineInterpolation( q1, q2, q3, q4, t, true );
        const auto q = spline( q1, q2, q3, q4, t );
        const auto p = kvs::Quaternion::Rotate( m_position0, q );
        const auto u = calcUpVector( p );
        path.add( { d, p, u, l } );
    }

    return path;
}

} // end of namespace InSituVis
