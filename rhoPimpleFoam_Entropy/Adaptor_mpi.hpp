namespace local
{

namespace mpi
{

inline void Adaptor::setOutputSubImageEnabled(
    const bool enable,
    const bool enable_depth,
    const bool enable_alpha )
{
    m_enable_output_subimage = enable;
    m_enable_output_subimage_depth = enable_depth;
    m_enable_output_subimage_alpha = enable_alpha;
}

inline bool Adaptor::initialize()
{
    if ( !BaseClass::outputDirectory().create( m_world ) )
    {
        this->log() << "ERROR: " << "Cannot create output directories." << std::endl;
        return false;
    }

    const bool depth_testing = true;
    const auto width = BaseClass::imageWidth();
    const auto height = BaseClass::imageHeight();
    if ( !m_image_compositor.initialize( width, height, depth_testing ) )
    {
        this->log() << "ERROR: " << "Cannot initialize image compositor." << std::endl;
        return false;
    }

    BaseClass::screen().setSize( width, height );
    BaseClass::screen().create();

    return true;
}

inline bool Adaptor::finalize()
{
    if ( m_image_compositor.destroy() )
    {
        return BaseClass::finalize();
    }
    return false;
}

inline void Adaptor::exec( const BaseClass::SimTime sim_time )
{
    if ( this->isAnalysisStep() )
    {
        // Stack current time step.
        const auto step = static_cast<float>( BaseClass::timeStep() );
        BaseClass::tstepList().stamp( step );

        this->execPipeline();
        this->execRendering();
    }

    BaseClass::incrementTimeStep();
    BaseClass::clearObjects();
}

inline bool Adaptor::dump()
{
    auto& tstep_list = BaseClass::tstepList();
    auto& pipe_timer = BaseClass::pipeTimer();
    auto& rend_timer = BaseClass::rendTimer();
    auto& save_timer = BaseClass::saveTimer();
    auto& comp_timer = m_comp_timer;
    if ( tstep_list.title().empty() ) { tstep_list.setTitle( "Time step" ); }
    if ( pipe_timer.title().empty() ) { pipe_timer.setTitle( "Pipe time" ); }
    if ( rend_timer.title().empty() ) { rend_timer.setTitle( "Rend time" ); }
    if ( save_timer.title().empty() ) { save_timer.setTitle( "Save time" ); }
    if ( comp_timer.title().empty() ) { comp_timer.setTitle( "Comp time" ); }

    const std::string rank = kvs::String::From( this->world().rank(), 4, '0' );
    const std::string subdir = BaseClass::outputDirectory().name() + "/";
    kvs::StampTimerList timer_list;
    timer_list.push( tstep_list );
    timer_list.push( pipe_timer );
    timer_list.push( rend_timer );
    timer_list.push( save_timer );
    timer_list.push( comp_timer );
    if ( !timer_list.write( subdir + "vis_proc_time_" + rank + ".csv" ) ) return false;

    using Time = kvs::mpi::StampTimer;
    Time pipe_time_min( this->world(), pipe_timer ); pipe_time_min.reduceMin();
    Time pipe_time_max( this->world(), pipe_timer ); pipe_time_max.reduceMax();
    Time pipe_time_ave( this->world(), pipe_timer ); pipe_time_ave.reduceAve();
    Time rend_time_min( this->world(), rend_timer ); rend_time_min.reduceMin();
    Time rend_time_max( this->world(), rend_timer ); rend_time_max.reduceMax();
    Time rend_time_ave( this->world(), rend_timer ); rend_time_ave.reduceAve();
    Time save_time_min( this->world(), save_timer ); save_time_min.reduceMin();
    Time save_time_max( this->world(), save_timer ); save_time_max.reduceMax();
    Time save_time_ave( this->world(), save_timer ); save_time_ave.reduceAve();
    Time comp_time_min( this->world(), comp_timer ); comp_time_min.reduceMin();
    Time comp_time_max( this->world(), comp_timer ); comp_time_max.reduceMax();
    Time comp_time_ave( this->world(), comp_timer ); comp_time_ave.reduceAve();

    if ( !this->world().isRoot() ) return true;

    pipe_time_min.setTitle( pipe_timer.title() + " (min)" );
    pipe_time_max.setTitle( pipe_timer.title() + " (max)" );
    pipe_time_ave.setTitle( pipe_timer.title() + " (ave)" );
    rend_time_min.setTitle( rend_timer.title() + " (min)" );
    rend_time_max.setTitle( rend_timer.title() + " (max)" );
    rend_time_ave.setTitle( rend_timer.title() + " (ave)" );
    save_time_min.setTitle( save_timer.title() + " (min)" );
    save_time_max.setTitle( save_timer.title() + " (max)" );
    save_time_ave.setTitle( save_timer.title() + " (ave)" );
    comp_time_min.setTitle( comp_timer.title() + " (min)" );
    comp_time_max.setTitle( comp_timer.title() + " (max)" );
    comp_time_ave.setTitle( comp_timer.title() + " (ave)" );

    timer_list.clear();
    timer_list.push( tstep_list );
    timer_list.push( pipe_time_min );
    timer_list.push( pipe_time_max );
    timer_list.push( pipe_time_ave );
    timer_list.push( rend_time_min );
    timer_list.push( rend_time_max );
    timer_list.push( rend_time_ave );
    timer_list.push( save_time_min );
    timer_list.push( save_time_max );
    timer_list.push( save_time_ave );
    timer_list.push( comp_time_min );
    timer_list.push( comp_time_max );
    timer_list.push( comp_time_ave );

    const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
    return timer_list.write( basedir + "vis_proc_time.csv" );
}

float calcColorEntropy( const size_t width, const size_t height, const kvs::ValueArray<unsigned char>& color_buffer, const kvs::ValueArray<float>& depth_buffer ){
    kvs::ValueArray<size_t> histogram( 256 );
    for( size_t i = 0; i < 256; i++ ){
        histogram[i] = 0;
    }
    size_t n = 0;
    for( size_t i = 0; i < ( width * height ); i++ ){
        if( depth_buffer[i] < 1 ){
            histogram[ color_buffer[i] ] += 1;
            n += 1;
        }
    }

    float entropy = 0.0;
    for( size_t i = 0; i < 256; i++ ){
        const float p = static_cast<float>( histogram[i] ) / n;
        if( p > 0 ){
            entropy -= p * log( p ) / log( 2.0f );
        }
    }

    return entropy;
}

float calcDepthEntropy( const size_t width, const size_t height, const kvs::ValueArray<float>& depth_buffer )
{
    kvs::ValueArray<size_t> histogram( 256 );
    for( size_t i = 0; i < 256; i++ ){
        histogram[i] = 0;
    }
    size_t n = 0;
    for( size_t i = 0; i < ( width * height ); i++ ){
        if( depth_buffer[i] < 1 ){
            const size_t j = depth_buffer[i] * 256;
            histogram[j] += 1;
            n += 1;
        }
    }

    float entropy = 0.0;
    for( size_t i = 0; i < 256; i++ ){
        const float p = static_cast<float>( histogram[i] ) / n;
        if( p > 0 ){
            entropy -= p * log( p ) / log( 2.0f );
        }
    }

    return entropy;
}

inline void Adaptor::execRendering()
{
    m_rend_time = 0.0f;
    m_comp_time = 0.0f;
    float save_time = 0.0f;
    
    size_t max_index = 0;
    float max_value = -1;
    //kvs::ValueArray<size_t> max_index( { 0, 0, 0 } );
    //kvs::ValueArray<size_t> min_index( { 0, 0, 0 } );
    //kvs::ValueArray<float> max_value( { -1.0f, -1.0f, -1.0f } );
    //kvs::ValueArray<float> min_value( { -1.0f, -1.0f, -1.0f } );

    kvs::ValueArray<kvs::ColorImage> images( BaseClass::viewpoint().numberOfLocations() );
    //kvs::ValueArray<float> color_entropy( BaseClass::viewpoint().numberOfLocations() );
    //kvs::ValueArray<float> depth_entropy( BaseClass::viewpoint().numberOfLocations() );
    kvs::ValueArray<float> entropy( BaseClass::viewpoint().numberOfLocations() );

    {
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = this->readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( m_world.rank() == m_world.root() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    const auto size = BaseClass::outputImageSize( location );
                    const auto index = location.index;
                    const auto width = size.x();
                    const auto height = size.y();
                    const auto color_buffer = frame_buffer.color_buffer;
                    const auto depth_buffer = frame_buffer.depth_buffer;
                    kvs::ColorImage image( width, height, color_buffer );
                    //color_entropy[ index ] = calcColorEntropy( width, height, color_buffer, depth_buffer );
                    //depth_entropy[ index ] = calcDepthEntropy( width, height, depth_buffer );
                    entropy[ index ] = calcDepthEntropy( width, height, depth_buffer );
                    images[ index ] = image;
                    //image.write( this->outputFinalImageName( location ) );

                    if( ( max_value < 0 ) || ( max_value < entropy[ index ] ) ){
                        max_value = entropy[ index ];
                        max_index = index;
                    }

                    /*if( ( max_value[0] < 0 ) || ( max_value[0] < color_entropy[ index ] ) )
                    {
                        max_value[0] = color_entropy[ index ];
                        max_index[0] = index;
                    }
                    if( ( min_value[0] < 0 ) || ( min_value[0] > color_entropy[ index ] ) )
                    {
                        min_value[0] = color_entropy[index];
                        min_index[0] = index;
                    }
                    if( ( max_value[1] < 0 ) || ( max_value[1] < depth_entropy[index] ) )
                    {
                        max_value[1] = depth_entropy[index];
                        max_index[1] = index;
                    }
                    if( ( min_value[1] < 0 ) || ( min_value[1] > depth_entropy[index] ) )
                    {
                        min_value[1] = depth_entropy[index];
                        min_index[1] = index;
                    }*/
                }
            }
            timer.stop();
            save_time += BaseClass::saveTimer().time( timer );
        }

        /*const float p = 0.0f;
        for( size_t i = 0; i < BaseClass::viewpoint().numberOfLocations(); i++ )
        {
            //float norm_color_entropy = ( color_entropy[i] - min_value[0] ) / ( max_value[0] - min_value[0] );
            float norm_color_entropy = 0.0f;
            float norm_depth_entropy = ( depth_entropy[i] - min_value[1] ) / ( max_value[1] - min_value[1] );
            entropy[i] = p * norm_color_entropy + ( 1.0f - p ) * norm_depth_entropy;
            if( ( max_value[2] < 0 ) || ( max_value[2] < entropy[i] ) )
            {
                max_value[2] = entropy[i];
                max_index[2] = i;
            }
            if( ( min_value[2] < 0 ) || ( min_value[2] > entropy[i] ) )
            {
                min_value[2] = entropy[i];
                min_index[2] = i;
            }
        }*/

        setIndex( max_index ); 
        if ( m_world.rank() == m_world.root() ){
            const auto time = BaseClass::timeStep();
            const auto space = max_index;
            const auto output_time = kvs::String::From( time, 6, '0' );
            const auto output_space = kvs::String::From( space, 6, '0' );
            const auto output_basename = BaseClass::outputFilename();
            const auto output_filename = output_basename + "_" + output_time + "_" + output_space;
            const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
            images[ space ].write( filename );
        }

        /*
        //const auto m_dims = BaseClass::viewpoint().dims();
        const auto num_x = 18;
        const auto num_y = 9;
        const size_t l = 16;
        const size_t width = ( 1 + l ) * num_x + 1;
        const size_t height = ( 1 + l ) * num_y + 1;
        kvs::ColorImage heatmap( width, height );
        auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        //auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        //t.setRange( min_value[2], max_value[2] );
        t.setRange( 0.0f, 1.0f );
        const auto cmap = t.colorMap();

        for( size_t y = 0; y < num_y; y++ ){
            for( size_t x =0; x < num_x; x++ ){
                for( size_t i = 0; i < ( 1 + l ); i++ ){
                    heatmap.setPixel( ( 1 + l ) * x + i, ( 1 + l ) * y, kvs::RGBColor::Black() );
                    heatmap.setPixel( ( 1 + l ) * x, ( 1 + l ) * y + i, kvs::RGBColor::Black() );
                }
                auto e = entropy[ y * num_x + x ];
                if ( !( e > min_value[2] && max_value[2] > e ) )
                {
                    log() << "error" << std::endl;
                    log() << "e = " << e << std::endl;
                    log() << "min = " << min_value[2] << std::endl;
                    log() << "max = " << max_value[2] << std::endl;
                }
                auto c = cmap.at( e );
                for( size_t j = 0; j < l; j++ ){
                    for( size_t i = 0; i < l; i++ ){
                        heatmap.setPixel( ( 1 + l ) * x + 1 + i, ( 1 + l ) * y + 1 + j, cmap.at( entropy[ y * num_x + x ] ) );
                        //heatmap.setPixel( ( 1 + l ) * x + 1 + i, ( 1 + l ) * y + 1 + j, kvs::RGBColor::White() );
                    }
                }
            }
        }
        for( size_t i = 0; i < width; i++ ){
            heatmap.setPixel( i, height - 1, kvs::RGBColor( 0, 0, 0 ) );
        }
        for( size_t j = 0; j < height; j++ ){
            heatmap.setPixel( width - 1, j, kvs::RGBColor( 0, 0, 0 ) );
        }
        const auto output_filename_heatmap = "heatmap_entropy_" + output_time;
        const auto filename_heatmap = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename_heatmap + ".bmp";
        heatmap.write( filename_heatmap );
        */
    }
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( m_rend_time );
    m_comp_timer.stamp( m_comp_time );
}

inline void Adaptor::execRenderingAt( const size_t i )
{
    
    m_rend_time = 0.0f;
    m_comp_time = 0.0f;
    float save_time = 0.0f;

    kvs::ValueArray<kvs::ColorImage> images( BaseClass::viewpoint().numberOfLocations() );

    {
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            auto frame_buffer = this->readback( location );
            
            kvs::Timer timer( kvs::Timer::Start );
            
            if ( m_world.rank() == m_world.root() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    const auto size = BaseClass::outputImageSize( location );
                    const auto index = location.index;
                    const auto width = size.x();
                    const auto height = size.y();
                    const auto color_buffer = frame_buffer.color_buffer;
                    kvs::ColorImage image( width, height, color_buffer );
                    images[index] = image;
                }
            }
            
            timer.stop();
            save_time += BaseClass::saveTimer().time( timer );
        }
        
        if( m_world.rank() == m_world.root() )
        {
            const auto time = BaseClass::timeStep();
            const auto output_time = kvs::String::From( time, 6, '0' );
            const auto output_basename = BaseClass::outputFilename();
            const auto output_filename = output_basename + "_" + output_time;
            const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
            images[i].write( filename );
        }
    }
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( m_rend_time );
    m_comp_timer.stamp( m_comp_time );
}

inline Adaptor::FrameBuffer Adaptor::drawScreen( std::function<void(const FrameBuffer&)> func )
{
    // Draw and read-back image
    kvs::Timer timer_rend( kvs::Timer::Start );
    BaseClass::screen().draw();
    timer_rend.stop();
    m_rend_time += BaseClass::rendTimer().time( timer_rend );

    // Apply the func for partial rendering buffers before image composition.
    auto color_buffer = BaseClass::screen().readbackColorBuffer();
    auto depth_buffer = BaseClass::screen().readbackDepthBuffer();
    func( { color_buffer, depth_buffer } );

    // Image composition
    kvs::Timer timer_comp( kvs::Timer::Start );
    if ( !m_image_compositor.run( color_buffer, depth_buffer ) )
    {
        this->log() << "ERROR: " << "Cannot compose images." << std::endl;
    }
    timer_comp.stop();
    m_comp_time += m_comp_timer.time( timer_comp );

    return { color_buffer, depth_buffer };
}

inline std::string Adaptor::outputFinalImageName( const InSituVis::Viewpoint::Location& location )
{
    const auto time = BaseClass::timeStep();
    const auto space = location.index;
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_space = kvs::String::From( space, 6, '0' );

    const auto output_basename = BaseClass::outputFilename();
    const auto output_filename = output_basename + "_" + output_time + "_" + output_space;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
    return filename;
}

inline void Adaptor::outputSubImages(
    const FrameBuffer& frame_buffer,
    const InSituVis::Viewpoint::Location& location,
    const std::string& suffix )
{
    if ( m_enable_output_subimage )
    {
        const auto& color_buffer = frame_buffer.color_buffer;
        const auto& depth_buffer = frame_buffer.depth_buffer;

        // Color image
        const auto width = BaseClass::imageWidth();
        const auto height = BaseClass::imageHeight();
        kvs::ColorImage image( width, height, color_buffer );
        image.write( BaseClass::outputImageName( location, "_color_" + suffix ) );

        // Depth image
        if ( m_enable_output_subimage_depth )
        {
            kvs::GrayImage depth_image( width, height, depth_buffer );
            depth_image.write( BaseClass::outputImageName( location, "_depth_" + suffix ) );
        }

        // Alpha image
        if ( m_enable_output_subimage_alpha )
        {
            kvs::GrayImage alpha_image( width, height, color_buffer, 3 );
            alpha_image.write( BaseClass::outputImageName( location, "_alpha_" + suffix ) );
        }
    }
}

inline Adaptor::DepthBuffer Adaptor::backgroundDepthBuffer()
{
    const auto width = BaseClass::screen().width();
    const auto height = BaseClass::screen().height();
    DepthBuffer buffer( width * height );
    buffer.fill(0);
    return buffer;
}

inline Adaptor::FrameBuffer Adaptor::readback( const InSituVis::Viewpoint::Location& location )
{
    switch ( location.direction )
    {
    case InSituVis::Viewpoint::Direction::Uni: return this->readback_uni_buffer( location );
    case InSituVis::Viewpoint::Direction::Omni: return this->readback_omn_buffer( location );
    case InSituVis::Viewpoint::Direction::Adaptive: return this->readback_adp_buffer( location );
    default: return { BaseClass::backgroundColorBuffer(), this->backgroundDepthBuffer() };
    }
}

inline Adaptor::FrameBuffer Adaptor::readback_uni_buffer( const InSituVis::Viewpoint::Location& location )
{
    const auto p = location.position;
    const auto a = location.look_at;
    const auto p_rtp = location.position_rtp;
    if ( p == a )
    {
        kvs::Timer timer_rend( kvs::Timer::Start );
        auto color_buffer = BaseClass::backgroundColorBuffer();
        auto depth_buffer = this->backgroundDepthBuffer();
        timer_rend.stop();
        m_rend_time += BaseClass::rendTimer().time( timer_rend );
        return { color_buffer, depth_buffer };
    }
    else
    {
        auto* camera = BaseClass::screen().scene()->camera();
        auto* light = BaseClass::screen().scene()->light();

        // Backup camera and light info.
        const auto p0 = camera->position();
        const auto a0 = camera->lookAt();
        const auto u0 = camera->upVector();

        //Draw the scene.
        kvs::Vec3 pp_rtp;
        if( p_rtp[1] > kvs::Math::pi / 2 ){
            pp_rtp = p_rtp - kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        else{
            pp_rtp = p_rtp + kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        const float pp_x = pp_rtp[0] * std::sin( pp_rtp[1] ) * std::sin( pp_rtp[2] );
        const float pp_y = pp_rtp[0] * std::cos( pp_rtp[1] );
        const float pp_z = pp_rtp[0] * std::sin( pp_rtp[1] ) * std::cos( pp_rtp[2] );
        kvs::Vec3 pp;
        if( p_rtp[1] > kvs::Math::pi / 2 ){
            pp = kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
        else{
            pp = -1 * kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
        const auto u = pp;
        camera->setPosition( p, a, u );
        light->setPosition( p );
        const auto buffer = this->drawScreen();


        // Restore camera and light info.
        camera->setPosition( p0, a0, u0 );
        light->setPosition( p0 );

        return buffer;
    }
}

inline Adaptor::FrameBuffer Adaptor::readback_omn_buffer( const InSituVis::Viewpoint::Location& location )
{
    using SphericalColorBuffer = InSituVis::SphericalBuffer<kvs::UInt8>;
    using SphericalDepthBuffer = InSituVis::SphericalBuffer<kvs::Real32>;

    auto* camera = BaseClass::screen().scene()->camera();
    auto* light = BaseClass::screen().scene()->light();

    // Backup camera and light info.
    const auto fov = camera->fieldOfView();
    const auto front = camera->front();
    const auto cp = camera->position();
    const auto ca = camera->lookAt();
    const auto cu = camera->upVector();
    const auto lp = light->position();
    const auto& p = location.position;

    // Draw the scene.
    camera->setFieldOfView( 90.0 );
    camera->setFront( 0.1 );
    light->setPosition( p );

    float rend_time = 0.0f;
    float comp_time = 0.0f;
    SphericalColorBuffer color_buffer( BaseClass::screen().width(), BaseClass::screen().height() );
    SphericalDepthBuffer depth_buffer( BaseClass::screen().width(), BaseClass::screen().height() );
    for ( size_t i = 0; i < SphericalColorBuffer::Direction::NumberOfDirections; i++ )
    {
        // Rendering.
        const auto d = SphericalColorBuffer::Direction(i);
        const auto dir = SphericalColorBuffer::DirectionVector(d);
        const auto up = SphericalColorBuffer::UpVector(d);
        camera->setPosition( p, p + dir, up );
        const auto buffer = this->drawScreen(
            [&] ( const FrameBuffer& frame_buffer )
            {
                // Output rendering image (partial rendering image) for each direction
                const auto dname = SphericalColorBuffer::DirectionName(d);
                this->outputSubImages( frame_buffer, location, dname );
            } );

        color_buffer.setBuffer( d, buffer.color_buffer );
        depth_buffer.setBuffer( d, buffer.depth_buffer );
    }

    m_rend_time = rend_time;
    m_comp_time = comp_time;

    // Restore camera and light info.
    camera->setFieldOfView( fov );
    camera->setFront( front );
    camera->setPosition( cp, ca, cu );
    light->setPosition( lp );

    // Return frame buffer
    return { color_buffer.stitch<4>(), depth_buffer.stitch<1>() };
}

inline Adaptor::FrameBuffer Adaptor::readback_adp_buffer( const InSituVis::Viewpoint::Location& location )
{
    const auto* object = BaseClass::screen().scene()->objectManager();
    return BaseClass::isInsideObject( location.position, object ) ?
        this->readback_omn_buffer( location ) :
        this->readback_uni_buffer( location );
}

} // end of namespace mpi

} // end of namespace InSituVis
