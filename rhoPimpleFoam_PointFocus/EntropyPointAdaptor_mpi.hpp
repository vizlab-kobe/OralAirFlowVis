#include <kvs/ColorImage>
#include <kvs/RGBColor>

namespace local
{

namespace mpi
{

inline void EntropyPointAdaptor::setOutputEvaluationImageEnabled(
    const bool enable,
    const bool enable_depth )
{
    m_enable_output_evaluation_image = enable;
    m_enable_output_evaluation_image_depth = enable_depth;
}

inline bool EntropyPointAdaptor::isEntropyStep()
{
    return BaseClass::timeStep() % ( BaseClass::analysisInterval() * Controller::entropyInterval() ) == 0;
}

inline bool EntropyPointAdaptor::isFinalTimeStep()
{
    return BaseClass::timeStep() == m_final_time_step;
}

inline bool EntropyPointAdaptor::dump()
{
    bool ret = true;
    if ( BaseClass::world().isRoot() )
    {
        if ( m_entr_timer.title().empty() ) { m_entr_timer.setTitle( "Ent time" ); }
        kvs::StampTimerList timer_list;
        timer_list.push( m_entr_timer );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        ret = timer_list.write( basedir + "ent_proc_time.csv" );

        this->output_path_entropies( Controller::pathEntropies() );
        this->output_path_positions( Controller::pathPositions() );
    }

    return BaseClass::dump() && ret;
}

inline void EntropyPointAdaptor::exec( const BaseClass::SimTime sim_time )
{
    Controller::setCacheEnabled( BaseClass::isAnalysisStep() );
    Controller::push( BaseClass::objects() );

    BaseClass::incrementTimeStep();
    if( this->isFinalTimeStep())
    {
        Controller::setFinalStep( true );
        const auto dummy = Data();
        Controller::push( dummy );
    }
    BaseClass::clearObjects();
}

inline void EntropyPointAdaptor::execRendering()
{
    BaseClass::setRendTime( 0.0f );
    BaseClass::setCompTime( 0.0f );
    float save_time = 0.0f;
    float entr_time = 0.0f;

    float max_entropy = -1.0f;
    int max_index = 0;

    std::vector<float> entropies;
    std::vector<BaseClass::FrameBuffer> frame_buffers;

    if ( this->isEntropyStep() )
    {
        // Entropy evaluation
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = BaseClass::readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( BaseClass::world().isRoot() )
            {
                const auto entropy = Controller::entropy( frame_buffer );
                entropies.push_back( entropy );
                frame_buffers.push_back( frame_buffer );

                if ( entropy > max_entropy )
                {
                    max_entropy = entropy;
                    max_index = location.index;
                }

                //this->output_color_image( location, frame_buffer );
            }
            timer.stop();
            entr_time += BaseClass::saveTimer().time( timer );
        }

        // Distribute the index indicates the max entropy image
        BaseClass::world().broadcast( max_index );
        BaseClass::world().broadcast( max_entropy );
        const auto max_position = BaseClass::viewpoint().at( max_index ).position;
        const auto max_rotation = BaseClass::viewpoint().at( max_index ).rotation;
        Controller::setMaxIndex( max_index );
        Controller::setMaxRotation( max_rotation );
        Controller::setMaxEntropy( max_entropy );

        if( Controller::previousData().empty() )
        {
            Controller::setMaxPositionStart( max_position );
            Controller::setMaxPositionMiddle( max_position );
            Controller::setMaxPositionEnd( max_position );
        }
        else
        {
            Controller::setMaxPositionStart( Controller::maxPositionMiddle() );
            Controller::setMaxPositionMiddle( Controller::maxPositionEnd() );
            Controller::setMaxPositionEnd( max_position );
        }



        // Output the rendering images and the heatmap of entropies.
        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().isRoot() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                const auto index = Controller::maxIndex();
                const auto& location = BaseClass::viewpoint().at( index );
                const auto& frame_buffer = frame_buffers[ index ];
                auto m_coord = regionEntropy(frame_buffer,4,4);
                this->output_color_image( location, frame_buffer );
                //this->output_depth_image( location, frame_buffer );
                this->output_entropy_table( entropies );
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    else
    {
        auto rotation = Controller::erpRotation();
        const size_t i = 999999;
        const auto d = InSituVis::Viewpoint::Direction::Uni;
        const auto p = kvs::Quaternion::Rotate( kvs::Vec3( { 0.0f, 12.0f, 0.0f } ), rotation );
        const auto u = kvs::Quaternion::Rotate( kvs::Vec3( { 0.0f, 0.0f, -1.0f } ), rotation );
        const auto l = kvs::Vec3( { 0.0f, 0.0f, 0.0f } );
        const auto location = InSituVis::Viewpoint::Location( i, d, p, u, rotation, l );
        auto frame_buffer = BaseClass::readback( location );
        const auto path_entropy = Controller::entropy( frame_buffer );
        Controller::setMaxEntropy( path_entropy );

        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().rank() == BaseClass::world().root() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                this->output_color_image( location, frame_buffer );
                //this->output_depth_image( location, frame_buffer );
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    m_entr_timer.stamp( entr_time );
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( BaseClass::rendTime() );
    BaseClass::compTimer().stamp( BaseClass::compTime() );
}

inline EntropyPointAdaptor::coord EntropyPointAdaptor::regionEntropy(const BaseClass::FrameBuffer& frame_buffer, int M, int N){
    coord m_coord;
    float max = -1.0f;
    const auto width = BaseClass::imageWidth();
    const auto height = BaseClass::imageHeight();
    int w_width = width/M;
    int h_height = height/N;
    kvs::ValueArray<size_t> l_histogram( 256 );
    kvs::ValueArray<size_t> d_histogram( 256 );
    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    const auto& color_buffer = frame_buffer.color_buffer;
    for(int i=0; i<N; i++){
        for(int j=0; j<M; j++){
            l_histogram.fill( 0 );
            d_histogram.fill( 0 );
            for(int k=h_height*i; k<h_height*(i+1) ;k++ ){
                for(int l=w_width*j; l<w_width*(j+1); l++){
                    auto depth = depth_buffer[width*k+l];
                    auto r = color_buffer[4*(width*k+l)];
                    auto g = color_buffer[4*(width*k+l)+1];
                    auto b = color_buffer[4*(width*k+l)+2];
                    auto lab = kvs::LabColor( kvs::RGBColor( r, g, b ) );
                    if ( depth < 1.0f ){
                        int m1 = static_cast<int>( lab.l() / 100 * 256 );
                        if( m1 > 255 ) m1 = 255;
                        l_histogram[m1] += 1;
                        n += 1;
                    }
                    if ( depth < 1.0f ){
                        const auto m2 = static_cast<int>( depth * 256 );
                        d_histogram[m2] += 1;
                        n += 1;
                    }
                }
            }
            float entropy = 0.0f;
            const auto log2 = std::log( 2.0f );
            for ( const auto h : l_histogram ){
                const auto p1 = static_cast<float>( h ) / n;
                if ( p1 > 0.0f ) { entropy -= p1 * std::log( p1 ) / log2; }
            }
            for ( const auto h : d_histogram ){
                const auto p2 = static_cast<float>( h ) / n;
                if ( p2 > 0.0f ) { entropy -= p2 * std::log( p2 ) / log2; }
            }
            if(max<entropy){
                m_coord.x = w_width*j+(w_width/2);
                m_coord.y = h_height*i+(h_height/2);
                m_coord.depth_buffer = depth_buffer[width*m_coord.x+m_coord.y];
            }
        }
    }
    return m_coord;
}
inline void EntropyPointAdaptor::process( const Data& data )
{
    BaseClass::execPipeline( data );
    this->execRendering();
}

inline void EntropyPointAdaptor::process( const Data& data, const kvs::Quaternion& rotation )
{
    const auto current_step = BaseClass::timeStep();
    {
        // Reset time step, which is used for output filename,
        // for visualizing the stacked dataset.
        const auto L_crr = Controller::dataQueue().size();
        if ( L_crr > 0 )
        {
            const auto l = BaseClass::analysisInterval();
            const auto step = current_step - L_crr * l;
            BaseClass::setTimeStep( step );
        }

        // Stack current time step.
        const auto step = static_cast<float>( BaseClass::timeStep() );
        BaseClass::tstepList().stamp( step );

        // Execute vis. pipeline and rendering.
        Controller::setErpRotation( rotation );
        BaseClass::execPipeline( data );
        this->execRendering();
    }
    BaseClass::setTimeStep( current_step );
}

inline void EntropyPointAdaptor::output_color_image(
    const InSituVis::Viewpoint::Location& location,
    const BaseClass::FrameBuffer& frame_buffer )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.color_buffer;
    kvs::ColorImage image( size.x(), size.y(), buffer );
    image.write( BaseClass::outputFinalImageName( location ) );
}

inline void EntropyPointAdaptor::output_depth_image(
    const InSituVis::Viewpoint::Location& location,
    const BaseClass::FrameBuffer& frame_buffer )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.depth_buffer;
    kvs::GrayImage image( size.x(), size.y(), buffer );
    image.write( BaseClass::outputFinalImageName( location ) );
}

inline void EntropyPointAdaptor::output_entropy_table(
    const std::vector<float> entropies )
{
    const auto time = BaseClass::timeStep();
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_filename =  "output_entropy_table_" + output_time;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream table( filename );

    const size_t dim = BaseClass::viewpoint().numberOfLocations();
    size_t dim_long = 1;
    auto y0 = BaseClass::viewpoint().at( 0 ).position[1];
    for( size_t i = 1; i < dim; i++ )
    {
        auto y = BaseClass::viewpoint().at( i ).position[1];
        if( y != y0 )
        {
            dim_long = i;
            break;
        }
    }

    for( size_t i = 0; i < dim; i++ )
    {
        table << entropies[i];
        table << ",";
        if( ( i + 1 ) % dim_long == 0 )
        {
            table << std::endl;
        }
    }

    table.close();
}

inline void EntropyPointAdaptor::output_path_entropies(
    const std::vector<float> path_entropies )
{
    const auto output_filename =  "output_path_entropies";
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream path_entropy( filename );
    const auto interval = BaseClass::analysisInterval();

    path_entropy << "Time,Entropy" << std::endl;
    for( size_t i = 0; i < path_entropies.size(); i++ )
    {
        path_entropy << interval * i << "," << path_entropies[i] << std::endl;
    }

    path_entropy.close();
}

inline void EntropyPointAdaptor::output_path_positions(
    const std::vector<float> path_positions )
{
    const auto output_filename =  "output_path_positions";
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream position( filename );
    const auto interval = BaseClass::analysisInterval();

    position << "Time,X,Y,Z" << std::endl;
    for( size_t i = 0; i < path_positions.size() / 3; i++ )
    {
        const auto x = path_positions[ 3 * i ];
        const auto y = path_positions[ 3 * i + 1 ];
        const auto z = path_positions[ 3 * i + 2 ];
        position << interval * i << "," << x << "," << y << "," << z << std::endl;
    }

    position.close();
}

} // end of namespace mpi

} // end of namespace InSituVis
