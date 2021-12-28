
namespace local
{

namespace mpi
{

inline void TimestepControlledAdaptor::exec( const BaseClass::SimTime sim_time )
{
    Controller::setCacheEnabled( BaseClass::isAnalysisStep() );
    Controller::push( BaseClass::objects() );

    BaseClass::incrementTimeStep();
    BaseClass::clearObjects();
}

inline kvs::Vec3 TimestepControlledAdaptor::calcProcess( const Data& data )
{
    BaseClass::execPipeline( data );
    BaseClass::execRendering();
    
    auto index = BaseClass::index();
    return BaseClass::viewpoint().at( index ).position;
}

inline void TimestepControlledAdaptor::process( const Data& data , const InSituVis::Viewpoint& path, const size_t i )
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
        BaseClass::execPipeline( data );
        const auto vp = BaseClass::viewpoint();
        BaseClass::setViewpoint( path );
        BaseClass::execRenderingAt( i );
        BaseClass::setViewpoint( vp );
    }
    BaseClass::setTimeStep( current_step );
}

} // end of namespace mpi

} // end of namespace InSituVis
