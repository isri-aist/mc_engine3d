#include <QGuiApplication>

#include <mc_engine3d/engine_interface.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char ** argv)
{
  // Engine3D inits
  QGuiApplication app(argc, argv);

  Q_UNUSED(app);

  std::string conf_file = "";
  po::options_description desc("mc_engine3d options");
  // clang-format off
   desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  mc_engine3d::EngineInterface engine(argc, argv);

  engine.init(conf_file);

  engine.run();

  return 0;
}
