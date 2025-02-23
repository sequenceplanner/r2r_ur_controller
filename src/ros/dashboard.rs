
// #[derive(Clone, PartialEq, Debug)]
// enum DashboardCommand {
//     Stop,
//     ResetProtectiveStop,
// }

// async fn handle_dashboard_commands(
//     mut service: impl Stream<Item = ServiceRequest<DBCommand::Service>> + Unpin,
//     dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
// )  -> Result<(), Box<dyn std::error::Error>> {
//     loop {
//         match service.next().await {
//             Some(req) => {
//                 println!("got dashboard command request: {:?}", req.message);
//                 let dbc = if req.message.cmd.contains("reset_protective_stop") {
//                     DashboardCommand::ResetProtectiveStop
//                 } else { // todo: implement more.
//                     DashboardCommand::Stop
//                 };

//                 let (sender, future) = oneshot::channel();
//                 dashboard_commands.try_send((dbc, sender)).expect("could not send");
//                 let ret = future.await;
//                 let ok = ret.is_ok();

//                 let resp = DBCommand::Response { ok };
//                 req.respond(resp).expect("could not send service response");
//             }
//             None => break,
//         }
//     }
//     Ok(())
// }
