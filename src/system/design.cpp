#include "design.hpp"

using namespace std;
Design::Design(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _applications, vector<int> _proc_mappings,
               vector<int>_proc_modes, vector<int> _next, vector<int> _sendingNext, 
               vector<int> _receivingNext, vector<int> _tdmaAlloc, 
               vector<int>  _sendbufferSz, vector<int>  _recbufferSz):
    mapping(_mapping),
    applications(_applications),
    no_entities(mapping->getNumberOfApps()),
    no_actors(applications->n_SDFActors()),
    no_channels(applications->n_SDFchannels()),
    no_processors(mapping->getPlatform()->nodes()),
    proc_mappings(_proc_mappings),
    proc_modes(_proc_modes),
    next(_next),
    sendingNext(_sendingNext),
    receivingNext(_receivingNext),
    tdmaAlloc(_tdmaAlloc),
    sendbufferSz(_sendbufferSz),
    recbufferSz(_recbufferSz)
    {
        check_inputs();
        for(size_t ii=0; ii<no_actors-1; ii++){
            if(applications->getSDFGraph(ii)+1 == applications->getSDFGraph(ii+1)){
            appIndex.push_back(ii);
            }
        }
        appIndex.push_back(no_actors-1);
        init_vectors();
    }
Design::Design(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _applications, vector<int> _proc_mappings,
               vector<int>_proc_modes, vector<int> _next, vector<int> _sendingNext, 
               vector<int> _receivingNext, vector<int> _tdmaAlloc):
    mapping(_mapping),
    applications(_applications),
    no_entities(mapping->getNumberOfApps()),
    no_actors(applications->n_SDFActors()),
    no_channels(applications->n_SDFchannels()),
    no_processors(mapping->getPlatform()->nodes()),
    proc_mappings(_proc_mappings),
    proc_modes(_proc_modes),
    next(_next),
    sendingNext(_sendingNext),
    receivingNext(_receivingNext),
    tdmaAlloc(_tdmaAlloc)
    {
        check_inputs();
        for(size_t ii=0; ii<no_actors-1; ii++){
            if(applications->getSDFGraph(ii)+1 == applications->getSDFGraph(ii+1)){
            appIndex.push_back(ii);
            }
        }
        appIndex.push_back(no_actors-1);
        sendbufferSz.resize(no_channels,10),
        recbufferSz.resize(no_channels,1);
        init_vectors();    
    }    
void Design::check_inputs()
{
   if(proc_mappings.size() != no_actors)
       THROW_EXCEPTION(RuntimeException, "proc_mappings.size() != no_actors" );
   if(proc_modes.size() != no_processors)
       THROW_EXCEPTION(RuntimeException, "proc_modes.size() != no_processors" );
   if(next.size() != no_actors + no_processors)
       THROW_EXCEPTION(RuntimeException, "next.size() != no_actors + no_processors" );
   if(sendingNext.size() != no_channels + no_processors)
       THROW_EXCEPTION(RuntimeException, "no_channels + no_processors" );
   if(receivingNext.size() != no_channels + no_processors)
       THROW_EXCEPTION(RuntimeException, "no_channels + no_processors" );
}
void Design::constructMSAG() {
  //cout << "\tDesign::constructMSAG()" << endl;
   bool printDebug = false;//TODO: remove prints
  //first, figure out how many actors there will be in the MSAG, in order to
  //initialize channel-matrix and actor-vector for the state of SSE
  n_msagActors = no_actors;
  for(size_t i = 0; i < no_channels; i++){
    if(sendingTime[i] > 0){ //=> channel on interconnect
      n_msagActors += 3; //one blocking, one sending and one receiving actor
    }
  }

  b::graph_traits<boost_msag_des>::vertex_descriptor src, dst;
  b::graph_traits<boost_msag_des>::edge_descriptor _e;

  msaGraph.clear();
  receivingActors.clear();
  channelMapping.clear();
  receivingActors.insert(receivingActors.begin(), no_actors, -1); //pre-fill with -1

  //add all actors as vertices, and self-loops
  bool found;
  for(size_t n = 0; n < n_msagActors; n++){
    add_vertex(n, b_msag);
    //add self-edges
    src = vertex(n, b_msag);
    tie(_e, found) = add_edge(src, src, b_msag);
    if(n < no_actors){
      b::put(b::edge_weight, b_msag, _e, wcet[n]);
    } //else{}: delay for communication actors are added further down
    b::put(b::edge_weight2, b_msag, _e, 1);
  }
  //next: add edges to boost-msag

  int channel_count = 0;
  int n_msagChannels = 0; //to count the number of channels in the MSAG

  //building the throughput analysis graph
  /* Step 1a: check sendingTime-array for all messages and add block-, send- & receive-"actors" with back-edges (buffering)
   Step 1b: check for dependencies in application graph that are not covered in 1a
   Step 2: check for decided forward-path in next-array
   Step 3: close execution cycles with back-edges found in next-array (next[i], i>=no_actors)
   */
  for(size_t i = 0; i < sendingTime.size(); i++){
    if(sendingTime[i] > 0){ //Step1a: => channel on interconnect

      int block_actor = no_actors + channel_count;
      int send_actor = block_actor + 1;
      int rec_actor = send_actor + 1;
      //store mapping between block/send/rec_actor and channel i
      channelMapping.push_back(i); //[block_actor] = i;
      channelMapping.push_back(i); //[send_actor] = i;
      channelMapping.push_back(i); //[rec_actor] = i;
      //add the block actor as a successor of ch_src[i]
      SuccessorNode succB;
      succB.successor_key = block_actor;
      succB.delay = sendingLatency[i];
      succB.min_tok = 0;
      succB.max_tok = 0;
      succB.channel = i;

      //add to boost-msag
      //src = b::vertex(ch_src[i], b_msag);
      src = b::vertex(applications->getChannels()[i]->source, b_msag);
      dst = b::vertex(block_actor, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingLatency[i]);
      b::put(b::edge_weight2, b_msag, _e, 0);
      //delay-weight for self-loop on block-actor:
      tie(_e, found) = edge(dst, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingLatency[i]);

      n_msagChannels++;
      if(printDebug)
      {
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(applications->getChannels()[i]->source);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(applications->getChannels()[i]->source).push_back(succB);
        }else{      //no entry for ch_src[i] yet
          vector<SuccessorNode> succBv;
          succBv.push_back(succB);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(applications->getChannels()[i]->source, succBv));
        }
      }

      //add ch_src[i] as successor of the block actor, with buffer size as tokens
      SuccessorNode srcCh;
      srcCh.successor_key = applications->getChannels()[i]->source;
      srcCh.delay = wcet[applications->getChannels()[i]->source];
      srcCh.min_tok = sendbufferSz[i];
      srcCh.max_tok = sendbufferSz[i];

      //add to boost-msag
      src = b::vertex(block_actor, b_msag);
      dst = b::vertex(applications->getChannels()[i]->source, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, wcet[applications->getChannels()[i]->source]);
      b::put(b::edge_weight2, b_msag, _e, sendbufferSz[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(block_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(block_actor).push_back(srcCh);
        }else{      //no entry for block_actor yet
          vector<SuccessorNode> srcChv;
          srcChv.push_back(srcCh);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(block_actor, srcChv));
        }
      }
//###
      //add the send actor as a successor of the block actor
      SuccessorNode succS;
      succS.successor_key = send_actor;
      succS.delay = sendingTime[i];
      succS.min_tok = 0;
      succS.max_tok = 0;
      succS.channel = i;

      //add to boost-msag
      src = b::vertex(block_actor, b_msag);
      dst = b::vertex(send_actor, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingTime[i]);
      b::put(b::edge_weight2, b_msag, _e, 0);
      //delay-weight for self-loop on send-actor:
      tie(_e, found) = edge(dst, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingTime[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(block_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(block_actor).push_back(succS);
        }else{      //no entry for block_actor yet
          vector<SuccessorNode> succSv;
          succSv.push_back(succS);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(block_actor, succSv));
        }
      }

      //add the block actor as successor of the send actor, with one token (serialization)
      SuccessorNode succBS;
      succBS.successor_key = block_actor;
      succBS.delay = sendingLatency[i];
      succBS.min_tok = 1;
      succBS.max_tok = 1;
      succBS.channel = i;

      //add to boost-msag
      src = b::vertex(send_actor, b_msag);
      dst = b::vertex(block_actor, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingLatency[i]);
      b::put(b::edge_weight2, b_msag, _e, 1);

      n_msagChannels++;
      if(printDebug)
      {
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(send_actor);
        if(it != msaGraph.end()){ //send actor already has an entry in the map
          msaGraph.at(send_actor).push_back(succBS);
        }else{      //no entry for send_actor yet
          vector<SuccessorNode> succBSv;
          succBSv.push_back(succBS);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(send_actor, succBSv));
        }
      }

      //add receiving actor as successor of the send actor, with potential initial tokens
      SuccessorNode dstCh;
      dstCh.successor_key = rec_actor;
      dstCh.delay = receivingTime[i];
      dstCh.min_tok = applications->getChannels()[i]->initTokens;
      dstCh.max_tok = applications->getChannels()[i]->initTokens;
      dstCh.channel = i;
      dstCh.recOrder = receivingNext[i];

      //add to boost-msag
      src = b::vertex(send_actor, b_msag);
      dst = b::vertex(rec_actor, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, receivingTime[i]);
      b::put(b::edge_weight2, b_msag, _e, applications->getChannels()[i]->initTokens);
      //delay-weight for self-loop on rec-actor:
      tie(_e, found) = edge(dst, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, receivingTime[i]);

      n_msagChannels++;
      if(printDebug)
      {
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(send_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(send_actor).push_back(dstCh);
        }else{      //no entry for i yet
          vector<SuccessorNode> dstChv;
          dstChv.push_back(dstCh);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(send_actor, dstChv));
        }
      }

      //save the receiving actors for each actor (for next order)
      if(receivingActors[applications->getChannels()[i]->destination] == -1){ //first rec_actor for the dst
        receivingActors[applications->getChannels()[i]->destination] = rec_actor;
      }else{
        int curRec_actor_ch = channelMapping[receivingActors[applications->getChannels()[i]->destination] - no_actors];
        //if(receivingNext[curRec_actor_ch].assigned())
        {
          if(receivingNext[curRec_actor_ch] < (int) no_channels){
            if(applications->getChannels()[receivingNext[curRec_actor_ch]]->destination != applications->getChannels()[i]->destination){ //last rec_actor for this dst
              receivingActors[applications->getChannels()[i]->destination] = rec_actor;
            } //else
          }else{ //last rec_actor for this dst
            receivingActors[applications->getChannels()[i]->destination] = rec_actor;
          }
        }
        //if(receivingNext[channelMapping[rec_actor - no_actors]].assigned())
        {
          if(receivingNext[channelMapping[rec_actor - no_actors]] == receivingActors[applications->getChannels()[i]->destination]){
            receivingActors[applications->getChannels()[i]->destination] = rec_actor;
          }
        }
      }

      //add the send actor as a successor node of the receiving actor, with rec. buffer size - initial tokens
      SuccessorNode succRec;
      succRec.successor_key = send_actor;
      succRec.delay = sendingTime[i];
      succRec.min_tok = recbufferSz[i] - applications->getChannels()[i]->initTokens;
      succRec.max_tok = recbufferSz[i] - applications->getChannels()[i]->initTokens;
      succRec.channel = i;

      //add to boost-msag
      src = b::vertex(rec_actor, b_msag);
      dst = b::vertex(send_actor, b_msag);
      b::tie(_e, found) = b::add_edge(src, dst, b_msag);
      b::put(b::edge_weight, b_msag, _e, sendingTime[i]);
      b::put(b::edge_weight2, b_msag, _e, recbufferSz[i] - applications->getChannels()[i]->initTokens);

      n_msagChannels++;
      if(printDebug)
      {
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(rec_actor);
        if(it != msaGraph.end()){ //i already has an entry in the map
          msaGraph.at(rec_actor).push_back(succRec);
        }else{ //no entry for i yet
          vector<SuccessorNode> succRecv;
          succRecv.push_back(succRec);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(rec_actor, succRecv));
        }
      }

      channel_count += 3;
    }else if(sendingTime[i] == 0){ //Step 1b: add all edges from G to the MSAG
      //if(!sendingTime[i].assigned() || (sendingTime[i].assigned() && applications->getChannels()[i]->initTokens > 0) || (sendingTime[i].assigned() && !next[ch_src[i]].assigned())){
      if((applications->getChannels()[i]->initTokens > 0) ){
        //ch_src[i] -> ch_dst[i]: add channel destination as successor node of the channel source
        SuccessorNode _dst;
        _dst.successor_key = applications->getChannels()[i]->destination;
        _dst.delay = wcet[applications->getChannels()[i]->destination];
        _dst.min_tok = applications->getChannels()[i]->initTokens;
        _dst.max_tok = applications->getChannels()[i]->initTokens;
        _dst.channel = i;

        //add to boost-msag
        src = b::vertex(applications->getChannels()[i]->source, b_msag);
        dst = b::vertex(applications->getChannels()[i]->destination, b_msag);
        b::tie(_e, found) = b::add_edge(src, dst, b_msag);
        b::put(b::edge_weight, b_msag, _e, wcet[applications->getChannels()[i]->destination]);
        b::put(b::edge_weight2, b_msag, _e, applications->getChannels()[i]->initTokens);

        n_msagChannels++;
        if(printDebug)
        {
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(applications->getChannels()[i]->source);
          if(it != msaGraph.end()){ //i already has an entry in the map
            msaGraph.at(applications->getChannels()[i]->source).push_back(_dst);
          }else{ //no entry for i yet
            vector<SuccessorNode> dstv;
            dstv.push_back(_dst);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(applications->getChannels()[i]->source, dstv));
          }
        }
      }
    }
  }


  //put sendNext relations into the MSAG
  
  for(unsigned int i = 1; i < channelMapping.size(); i += 3){ //for all sending actors
    bool continues = true;
    bool nextFound = false;
    int nextCh;
    int x = channelMapping[i];
    int tokens = 0; //is channel to add a cycle-closing back-edge?
    while(!nextFound && continues){
      //if(sendingNext[x].assigned())
      {
        nextCh = sendingNext[x];
        if(nextCh >= (int) no_channels){ //end of chain found
          if(nextCh > (int) no_channels){
            nextCh = no_channels + ((nextCh - no_channels - 1) % no_processors);
          }else{
            nextCh = no_channels + no_processors - 1;
          }
          tokens = 1;
          //if(sendingNext[nextCh].assigned())
          {
            nextCh = sendingNext[nextCh];
            if(sendingTime[nextCh] > 0){
              nextFound = true;
            }else{
              x = nextCh;
            }
          }/*else{
            continues = false;
          }*/
        }else{ //not end of chain (nextCh < no_channels)
          if(sendingTime[nextCh] > 0){
            nextFound = true;
            if(tokens != 1)
              tokens = 0;
          }else{
            x = nextCh; //nextCh is not on interconnect. Continue with nextSend[nextCh].
          }
        }
      }/*else{
        continues = false;
      }*/
    }
    if(nextFound){
      //add send_actor of channel i -> block_actor of nextCh
      if(channelMapping[i] != nextCh){ //if found successor is not the channel's own block_actor (then it is already in the graph)
        int block_actor = getBlockActor(nextCh);

        //cout << "Next channel (send_actor of channel " << i << "): " << nextCh << endl;

        SuccessorNode succBS;
        succBS.successor_key = block_actor;
        succBS.delay = sendingLatency[nextCh];
        succBS.min_tok = tokens;
        succBS.max_tok = tokens;
        succBS.channel = nextCh;

        //add to boost-msag
        src = b::vertex(i + no_actors, b_msag);
        dst = b::vertex(block_actor, b_msag);
        b::tie(_e, found) = b::add_edge(src, dst, b_msag);
        b::put(b::edge_weight, b_msag, _e, sendingLatency[nextCh]);
        b::put(b::edge_weight2, b_msag, _e, tokens);

        n_msagChannels++;
        if(printDebug){
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i + no_actors);
          if(it != msaGraph.end()){ //send actor already has an entry in the map
            msaGraph.at(i + no_actors).push_back(succBS);
          }else{ //no entry for send_actor i yet
            vector<SuccessorNode> succBSv;
            succBSv.push_back(succBS);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(i + no_actors, succBSv));
          }
        }
      }
    }
  }

  //put recNext relations into the MSAG
  for(unsigned int i = 2; i < channelMapping.size(); i += 3){ //for all receiving actors
    bool nextFound = false;
    int nextCh;
    int x = channelMapping[i];

    //cout << "looking for recNext, channel " << i << endl;

    while(!nextFound){
      //if(receivingNext[x].assigned())
      {
        nextCh = receivingNext[x];
        if(nextCh >= (int) no_channels){ //end of chain found
          nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
          nextFound = true;
        }else{ //not end of chain (nextCh < no_channels)
          if(applications->getChannels()[nextCh]->destination != applications->getChannels()[channelMapping[i]]->destination){ //next rec actor belongs to other dst
            nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
            nextFound = true;
          }else{ //same dst
            if(sendingTime[nextCh] > 0){
              nextFound = true;
            }else{
              x = nextCh; //nextCh is not on interconnect. Continue with nextSend[nextCh].
            }
          }
        }
      }
      /*else{
        nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
        nextFound = true;
      }*/
    }

    //cout << "  found " << nextCh;

    SuccessorNode succRec;
    succRec.successor_key = nextCh == -1 ? applications->getChannels()[channelMapping[i]]->destination : getRecActor(nextCh);
    succRec.delay = nextCh == -1 ? wcet[applications->getChannels()[channelMapping[i]]->destination] : receivingTime[nextCh];
    succRec.min_tok = 0;
    succRec.max_tok = 0;
    if(nextCh != -1)
      succRec.channel = nextCh;

    //cout << " ( "<< succRec.successor_key <<")" << endl;

    //add to boost-msag
    src = b::vertex(i + no_actors, b_msag);
    dst = b::vertex(nextCh == -1 ? applications->getChannels()[channelMapping[i]]->destination : getRecActor(nextCh), b_msag);
    b::tie(_e, found) = b::add_edge(src, dst, b_msag);
    b::put(b::edge_weight, b_msag, _e, nextCh == -1 ? wcet[applications->getChannels()[channelMapping[i]]->destination] : receivingTime[nextCh]);
    b::put(b::edge_weight2, b_msag, _e, 0);

    n_msagChannels++;
    if(printDebug){
      unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i + no_actors);
      if(it != msaGraph.end()){ //send actor already has an entry in the map
        msaGraph.at(i + no_actors).push_back(succRec);
      }else{ //no entry for send_actor i yet
        vector<SuccessorNode> succRecv;
        succRecv.push_back(succRec);
        msaGraph.insert(pair<int, vector<SuccessorNode>>(i + no_actors, succRecv));
      }
    }
  }

  for(size_t i = 0; i < no_actors; i++){
    //Step 2
    if(next[i] < (int) no_actors){ //if next[i] is decided, the forward edge goes from i to next[i]
      int nextActor = next[i];

      //check whether nextActor has preceding rec_actor
      SuccessorNode nextA;
      if(receivingActors[nextActor] == -1){
        //add edge i -> nextActor
        nextA.successor_key = nextActor;
        nextA.delay = wcet[nextActor];
        nextA.min_tok = 0;
        nextA.max_tok = 0;

        //add to boost-msag
        src = b::vertex(i, b_msag);
        dst = b::vertex(nextActor, b_msag);
        b::tie(_e, found) = b::add_edge(src, dst, b_msag);
        b::put(b::edge_weight, b_msag, _e, wcet[nextActor]);
        b::put(b::edge_weight2, b_msag, _e, 0);
      }else{
        //add edge i -> receivingActor[nextActor]
        nextA.successor_key = receivingActors[nextActor];
        nextA.delay = receivingTime[channelMapping[receivingActors[nextActor] - no_actors]];
        nextA.min_tok = 0;
        nextA.max_tok = 0;
        nextA.channel = channelMapping[receivingActors[nextActor] - no_actors];

        //add to boost-msag
        src = b::vertex(i, b_msag);
        dst = b::vertex(receivingActors[nextActor], b_msag);
        b::tie(_e, found) = b::add_edge(src, dst, b_msag);
        b::put(b::edge_weight, b_msag, _e, receivingTime[channelMapping[receivingActors[nextActor] - no_actors]]);
        b::put(b::edge_weight2, b_msag, _e, 0);
      }

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i);
        if(it != msaGraph.end()){ //i already has an entry in the map
          msaGraph.at(i).push_back(nextA);
        }else{ //no entry for i yet
          vector<SuccessorNode> nextAv;
          nextAv.push_back(nextA);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(i, nextAv));
        }
      }

    }else if(next[i] >= (int) no_actors){ //next[i]>=no_actors
    //Step 3: add cycle-closing edge on each proc
      int firstActor = next[i];
      if(firstActor > (int) no_actors){
        firstActor = no_actors + ((firstActor - no_actors - 1) % no_processors);
      }else{
        firstActor = no_actors + no_processors - 1;
      }
      //if(next[firstActor].assigned())
      {
        firstActor = next[firstActor];

        //check whether firstActor has preceding rec_actor
        SuccessorNode first;
        if(receivingActors[firstActor] == -1){
          //add edge i -> firstActor
          first.successor_key = firstActor;
          first.delay = wcet[firstActor];
          first.min_tok = 1;
          first.max_tok = 1;

          //add to boost-msag
          src = b::vertex(i, b_msag);
          dst = b::vertex(firstActor, b_msag);
          b::tie(_e, found) = b::add_edge(src, dst, b_msag);
          b::put(b::edge_weight, b_msag, _e, wcet[firstActor]);
          b::put(b::edge_weight2, b_msag, _e, 1);
        }else{
          //add edge i -> receivingActor[firstActor]
          first.successor_key = receivingActors[firstActor];
          first.delay = receivingTime[channelMapping[receivingActors[firstActor] - no_actors]];
          first.min_tok = 1;
          first.max_tok = 1;
          first.channel = channelMapping[receivingActors[firstActor] - no_actors];

          //add to boost-msag
          src = b::vertex(i, b_msag);
          dst = b::vertex(receivingActors[firstActor], b_msag);
          b::tie(_e, found) = b::add_edge(src, dst, b_msag);
          b::put(b::edge_weight, b_msag, _e, receivingTime[channelMapping[receivingActors[firstActor] - no_actors]]);
          b::put(b::edge_weight2, b_msag, _e, 1);
        }

        n_msagChannels++;
        if(printDebug){
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i);
          if(it != msaGraph.end()){ //i already has an entry in the map
            msaGraph.at(i).push_back(first);
          }else{    //no entry for i yet
            vector<SuccessorNode> firstv;
            firstv.push_back(first);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(i, firstv));
          }
        }
      }
    }
  }

  if(printDebug){
    //printThroughputGraphAsDot(".");
  }
  
}


int Design::getBlockActor(int ch_id) const {
  auto it = find(channelMapping.begin(), channelMapping.end(), ch_id);
  if(it != channelMapping.end())
    return distance(channelMapping.begin(), it) + no_actors;

  return -1;
}

int Design::getSendActor(int ch_id) const {
  auto it = find(channelMapping.begin(), channelMapping.end(), ch_id);
  if(it != channelMapping.end())
    return distance(channelMapping.begin(), it) + no_actors + 1;

  return -1;
}

int Design::getRecActor(int ch_id) const {
  auto it = find(channelMapping.begin(), channelMapping.end(), ch_id);
  if(it != channelMapping.end())
    return distance(channelMapping.begin(), it) + no_actors + 2;

  return -1;
}

int Design::getApp(int msagActor_id) const {
  int id = msagActor_id;
  if(msagActor_id >= (int) no_actors){
    id = applications->getChannels()[channelMapping[msagActor_id - no_actors]]->destination;
  }
  for(size_t i = 0; i < no_actors; i++){
    if(id <= appIndex[i])
      return i;
  }
  return -1;
}

void checkApp_des(int app, unordered_map<int, set<int>>& coMappedApps, vector<int>& uncheckedApps, set<int>& res) {
  res.insert(app);
  uncheckedApps[app] = 0;
  for(auto& appl : coMappedApps[app])
    if(uncheckedApps[appl])
      checkApp_des(appl, coMappedApps, uncheckedApps, res);
}

void Design::calc_periods(){
    periods.clear();
    periods.resize(applications->n_SDFApps(), 0);
    vector<int> msagMap(applications->n_SDFApps(), 0);
    bool printDebug = true;
    
    if(applications->n_SDFApps() > 1){
    //check which application graphs are mapped to same processor (= combined into the same MSAG)
    vector<set<int>> result;
    unordered_map<int, set<int>> coMappedApps;
    vector<int> uncheckedApps(appIndex.size(), 1);
    for(size_t a = 0; a < appIndex.size(); a++){
      coMappedApps.insert(pair<int, set<int>>(a, set<int>()));
    }
    for(int i = 0; i < (int) no_actors; i++){
      if(next[i] < (int) no_actors){ //next[i] is decided and points to an application actor
        int actor = i;
        int nextActor = next[i];
        if(getApp(actor) != getApp(nextActor)){ //from different applications
          unordered_map<int, set<int>>::const_iterator it = coMappedApps.find(getApp(actor));
          if(it != coMappedApps.end()){ //i already has an entry in the map
            coMappedApps.at(getApp(actor)).insert(getApp(nextActor));
          }else{ //no entry for ch_src[i] yet
            set<int> coApp;
            coApp.insert(getApp(nextActor));
            coMappedApps.insert(pair<int, set<int>>(getApp(actor), coApp));
          }
          it = coMappedApps.find(getApp(nextActor));
          if(it != coMappedApps.end()){ //i already has an entry in the map
            coMappedApps.at(getApp(nextActor)).insert(getApp(actor));
          }else{ //no entry for ch_src[i] yet
            set<int> coApp;
            coApp.insert(getApp(actor));
            coMappedApps.insert(pair<int, set<int>>(getApp(nextActor), coApp));
          }
        }
      }
    }
//    if(coMappedApps.size() > 0){

      int sum_unchecked = 0;
      for(int x : uncheckedApps)
        sum_unchecked += x;
      while(sum_unchecked){

        for(auto& mapp : coMappedApps){
          if(printDebug){
            cout << "App " << mapp.first << " is" << (mapp.second.empty() ? " not " : " ") << "co-mapped with ";
            cout << (mapp.second.empty() ? string(" any other app") : tools::toString(mapp.second)) << endl;
          }

          if(uncheckedApps[mapp.first]){
            set<int> res;
            result.push_back(res);
            checkApp_des(mapp.first, coMappedApps, uncheckedApps, result.back());
          }
        }

        sum_unchecked = 0;
        for(int x : uncheckedApps)
          sum_unchecked += x;

      }

//    }else{
//      for(size_t i = 0; i < wc_period.size(); i++){
//        set<int> res;
//        res.insert(i);
//        result.push_back(res);
//      }
//    }
    for(size_t i = 0; i < result.size(); i++){
      b_msags.push_back(new boost_msag_des());
      for(auto it = result[i].begin(); it != result[i].end(); ++it){
        msagMap[*it] = i;
      }
    }
    constructMSAG(msagMap);

    if(printDebug){
      //if(next.assigned() && wcet.assigned())
      {
        cout << "trying to print " << b_msags.size() << " boost-msags." << endl;
        for(size_t t = 0; t < b_msags.size(); t++){
          cout << "Graph " << t << endl;
          cout << "  Vertices number: " << num_vertices(*b_msags[t]) << endl;
          cout << "  Edges number: " << num_edges(*b_msags[t]) << endl;
          string graphName = "boost_msag" + to_string(t);
          ofstream out;
          string outputFile = ".";
          outputFile += (outputFile.back() == '/') ? (graphName + ".dot") : ("/" + graphName + ".dot");
          out.open(outputFile.c_str());
          write_graphviz(out, *b_msags[t]);
          out.close();
          cout << "  Printed dot graph file " << outputFile << endl;
        }
        printThroughputGraphAsDot(".");
      }
    }

    vector<int> msag_mcrs;
    for(auto m : b_msags){
      using namespace boost;
      int max_cr; /// maximum cycle ratio
      typedef std::vector<graph_traits<boost_msag_des>::edge_descriptor> t_critCycl;
      t_critCycl cc; ///critical cycle
      property_map<boost_msag_des, vertex_index_t>::type vim = get(vertex_index, *m);
      property_map<boost_msag_des, edge_weight_t>::type ew1 = get(edge_weight, *m);
      property_map<boost_msag_des, edge_weight2_t>::type ew2 = get(edge_weight2, *m);

      //do MCR analysis
      max_cr = maximum_cycle_ratio(*m, vim, ew1, ew2, &cc);
      msag_mcrs.push_back(max_cr);
      if(printDebug){
        cout << "Period of app(s) " << tools::toString(result[msag_mcrs.size()-1]) << ": ";
        cout <<  max_cr << endl;
        cout << "Critical cycle:\n";
        for(t_critCycl::iterator itr = cc.begin(); itr != cc.end(); ++itr){
          cout << "(" << vim[source(*itr, b_msag)] << "," << vim[target(*itr, b_msag)] << ") ";
        }
        cout << endl;
      }
    }
    for(size_t i = 0; i < msag_mcrs.size(); i++){
      for(auto r: result[i]){
        periods[r] = msag_mcrs[i];        
      }
    }

  }else{ //only a single application
    constructMSAG();
    using namespace boost;
    int max_cr; /// maximum cycle ratio
    typedef std::vector<graph_traits<boost_msag_des>::edge_descriptor> t_critCycl;
    t_critCycl cc; ///critical cycle

    property_map<boost_msag_des, vertex_index_t>::type vim = get(vertex_index, b_msag);
    property_map<boost_msag_des, edge_weight_t>::type ew1 = get(edge_weight, b_msag);
    property_map<boost_msag_des, edge_weight2_t>::type ew2 = get(edge_weight2, b_msag);
    //do MCR analysis
    max_cr = maximum_cycle_ratio(b_msag, vim, ew1, ew2, &cc);
    periods[0] = max_cr;

    if(printDebug)
    {
      //if(next.assigned() && wcet.assigned())
      {
        string graphName = "boost_msag_des";
        ofstream out;
        string outputFile = ".";
        outputFile += (outputFile.back() == '/') ? (graphName + ".dot") : ("/" + graphName + ".dot");
        out.open(outputFile.c_str());
        write_graphviz(out, b_msag);
        out.close();
        printThroughputGraphAsDot(".");
      }

      cout << "Maximum cycle ratio is " << max_cr << endl;
      cout << "Critical cycle:\n";
      for(t_critCycl::iterator itr = cc.begin(); itr != cc.end(); ++itr){
        cout << "(" << vim[source(*itr, b_msag)] << "," << vim[target(*itr, b_msag)] << ") ";
      }
      cout << endl;
    }
}
}

void Design::init_vectors(){
    memCons.resize(no_processors,0);
    wcet.resize(no_actors,0);
    
    for(size_t i=0;i<no_channels;i++){
        /// sendingTime 
        int src_i = applications->getChannels()[i]->source;
        int dest_i = applications->getChannels()[i]->destination;
        int proc_src_i = proc_mappings[src_i];
        int proc_dest_i = proc_mappings[dest_i];    
        if(proc_src_i != proc_dest_i){
             sendingTime.push_back(mapping->wcTransferTimes(i)[tdmaAlloc[proc_src_i]]);
             /// sendingLatency
             sendingLatency.push_back(mapping->wcBlockingTimes()[tdmaAlloc[proc_src_i]]);    
             /// memCons
             memCons[proc_src_i] += applications->getChannels()[i]->messageSize;
             memCons[proc_dest_i] += applications->getChannels()[i]->messageSize;
        }else{
            sendingTime.push_back(0);///zero sending time if on the same processor
            sendingLatency.push_back(0);
            sendbufferSz[i] = 0; ///also no need for buffer
            /// memCons
            memCons[proc_src_i] += applications->getChannels()[i]->messageSize;            
        }
        ///(iv) receivingTime -> zero for TDMA-based platform
        receivingTime.push_back(0);    
    }
    for(size_t i=0;i<no_actors;i++){
        int proc_i = proc_mappings[i];
        int mode_i = proc_modes[proc_i];
        memCons[proc_i] += mapping->memConsCode(i, proc_i) +
                           mapping->memConsData(i, proc_i);
        ///(vi) wcet             
        wcet[i] = mapping->getWCET(i, proc_i, mode_i);
        if(wcet[i] < 0)
            THROW_EXCEPTION(RuntimeException, "wcet[i] < 0 actor"+
                            tools::toString(i)+"proc:"+tools::toString(proc_i)
                            +"mode:"+tools::toString(mode_i) );
    }
}


void Design::printThroughputGraphAsDot(const string &dir) const {

  string graphName = "throughputGraph";
  ofstream out;
  string outputFile = dir;
  outputFile += (outputFile.back() == '/') ? (graphName + ".dot") : ("/" + graphName + ".dot");
  out.open(outputFile.c_str());

  out << "digraph " << graphName << " {" << endl;
  out << "    size=\"7,10\";" << endl;
  //out << "    rankdir=\"LR\"" << endl;

  //Output actors
  for(size_t i = 0; i < n_msagActors; i++){
    string actorName;
    int col = 0;
    if(i < no_actors){
      actorName = "actor_" + to_string(i);
      col = (getApp(i) + 1) % 32;
    }else if(i >= no_actors && (i - no_actors) % 3 == 0){ //blocking node
      actorName = "block_ch" + to_string(channelMapping[i - no_actors]);
      col = -1;
    }else if(i >= no_actors && (i - no_actors) % 3 == 1){ //sending node
      actorName = "send_ch" + to_string(channelMapping[i - no_actors]);
      col = -2;
    }else if(i >= no_actors && (i - no_actors) % 3 == 2){ //receiving node
      actorName = "rec_ch" + to_string(channelMapping[i - no_actors]);
      col = -3;
    }

    out << "    " << actorName << " [ label=\"" << actorName << "\"";
    out << ", style=filled";
    out << ", fillcolor=\"";
    if(col == -1){
      out << "#999999";
    }else if(col == -2){
      out << "#CCCCCC";
    }else if(col == -3){
      out << "#FFFFFF";
    }else if(col > 0 && col < 13){
      // (See ColorBrewer license)
      out << "/set312/" << col;
    }else if(col > 12 && col < 24){
      // (See ColorBrewer license)
      out << "/spectral11/" << col - 12;
    }else{ //col>23
           // (See ColorBrewer license)
      out << "/set19/" << col - 23;
    }
    out << "\"";
    out << "];" << endl;
  }
  out << endl;

  for(auto it = msaGraph.begin(); it != msaGraph.end(); ++it){
    string srcName;
    size_t node = it->first;
    if(node < no_actors){
      srcName = "actor_" + to_string(node);
    }else if(node >= no_actors && (node - no_actors) % 3 == 0){ //blocking node
      srcName = "block_ch" + to_string(channelMapping[node - no_actors]);
    }else if(node >= no_actors && (node - no_actors) % 3 == 1){ //sending node
      srcName = "send_ch" + to_string(channelMapping[node - no_actors]);
    }else if(node >= no_actors && (node - no_actors) % 3 == 2){ //receiving node
      srcName = "rec_ch" + to_string(channelMapping[node - no_actors]);
    }
    vector<SuccessorNode> succs = (vector<SuccessorNode> ) (it->second);
    for(auto itV = succs.begin(); itV != succs.end(); ++itV){
      size_t node2 = ((SuccessorNode) (*itV)).successor_key;
      string dstName;
      if(node2 < no_actors){
        dstName = "actor_" + to_string(node2);
      }else if(node2 >= no_actors && (node2 - no_actors) % 3 == 0){ //blocking node
        dstName = "block_ch" + to_string(channelMapping[node2 - no_actors]);
      }else if(node2 >= no_actors && (node2 - no_actors) % 3 == 1){ //sending node
        dstName = "send_ch" + to_string(channelMapping[node2 - no_actors]);
      }else if(node2 >= no_actors && (node2 - no_actors) % 3 == 2){ //receiving node
        dstName = "rec_ch" + to_string(channelMapping[node2 - no_actors]);
      }
      int tok = ((SuccessorNode) (*itV)).max_tok;
      // Initial tokens on channel?
      if(tok != 0){
        string label;
        out << "    " << srcName << " -> " << dstName;
        if(node >= no_actors && (node - no_actors) % 3 == 0 && node2 < no_actors){
          label = "send_buff (ch" + to_string(channelMapping[node - no_actors]) + ")";
          out << " [ label=\"" << label << "\"];" << endl;
        }else if(node >= no_actors && (node - no_actors) % 3 == 2 && node2 >= no_actors && (node2 - no_actors) % 3 == 1){
          label = "rec_buff (ch" + to_string(channelMapping[node2 - no_actors]) + ")";
          out << " [ label=\"" << label << "\"];" << endl;
        }else{
          label = to_string(tok);
          out << " [ label=\"" << "init(";
          out << label << ")" << "\"];" << endl;
        }

      }else{
        out << "    " << srcName << " -> " << dstName << ";" << endl;
      }
    }
  }

  for(size_t i = 0; i < no_actors; i++){
    bool close = false;
    if(next[i] < (int) no_actors){
      out << "    { rank=same; ";
      out << "actor_" + to_string(i) << " ";
      out << "actor_" + to_string(next[i]) << " ";
      close = true;
    }
    for(size_t j = 0; j < no_channels; j++){
      if((int)i == applications->getChannels()[j]->destination && getRecActor(j) != -1){
        if(!close){
          out << "    { rank=same; ";
          out << "actor_" + to_string(i) << " ";
        }
        out << "rec_ch" + to_string(j) << " ";
        close = true;
      }
    }
    if(close)
      out << "}" << endl;
  }

  out << "}" << endl;

  out.close();

  cout << "  Printed dot graph file " << outputFile << endl;

}
struct myGraph {
  typedef typename b::property_map<boost_msag_des, vertex_actorida_t>::const_type IdMap;
  map<int, b::graph_traits<boost_msag_des>::vertex_descriptor> vertices;

  void addVertex(int id, b::graph_traits<boost_msag_des>::vertex_descriptor vertex) {
    vertices[id] = vertex;
  }
  int getId(b::graph_traits<boost_msag_des>::vertex_descriptor vertex, const boost_msag_des& graph) {
    IdMap id = b::get(vertex_actorida, graph);
    auto id_ = b::get(id, vertex);
    return id_;
  }
  b::graph_traits<boost_msag_des>::vertex_descriptor getVertex(int id) {
    return vertices[id];
  }
};

void Design::constructMSAG(vector<int> &msagMap) {
  bool printDebug = false;
  if(printDebug)
    cout << "\tThroughputMCR::constructMSAG(vector<int> &msagMap)" << endl;

  msaGraph.clear();
  receivingActors.clear();
  channelMapping.clear();
  receivingActors.insert(receivingActors.begin(), no_actors, -1); //pre-fill with -1
  //to identify for each msag-actor, which msag it belongs to
  vector<int> msagId;

  //first, figure out how many actors there will be in the MSAG
  n_msagActors = no_actors;
  for(size_t i = 0; i < sendingTime.size(); i++){
    if(sendingTime[i] > 0){ //=> channel on interconnect
      n_msagActors += 3; //one blocking, one sending and one receiving actor
      //store mapping between block/send/rec_actor and channel i
      channelMapping.push_back(i); //[block_actor] = i;
      channelMapping.push_back(i); //[send_actor] = i;
      channelMapping.push_back(i); //[rec_actor] = i;
    }
  }

  for(size_t i = 0; i < n_msagActors; i++){
    msagId.push_back(msagMap[getApp(i)]);
  }

  b::graph_traits<boost_msag_des>::vertex_descriptor src, dst;
  b::graph_traits<boost_msag_des>::edge_descriptor _e;
  myGraph g;

  //add all actors as vertices, and self-loops
  bool found;
  for(size_t n = 0; n < n_msagActors; n++){
    boost_msag_des& curr_graph = *b_msags[msagId[n]];

    src = add_vertex(curr_graph);
    b::put(vertex_actorida, curr_graph, src, n);
    g.addVertex(n, src);
    //add self-edges
    tie(_e, found) = add_edge(src, src, curr_graph);
    if(n < no_actors){
      b::put(b::edge_weight, curr_graph, _e, wcet[n]);
    } //else{}: delay for communication actors are added further down
    b::put(b::edge_weight2, curr_graph, _e, 1);
  }
  //next: add edges to boost-msag

  int channel_count = 0;
  int n_msagChannels = 0; //to count the number of channels in the MSAG

  //building the throughput analysis graph
  /* Step 1a: check sendingTime-array for all messages and add block-, send- & receive-"actors" with back-edges (buffering)
   Step 1b: check for dependencies in application graph that are not covered in 1a
   Step 2: check for decided forward-path in next-array
   Step 3: close execution cycles with back-edges found in next-array (next[i], i>=no_actors)
   */
  for(size_t i = 0; i < sendingTime.size(); i++){
    if(sendingTime[i] > 0){ //Step1a: => channel on interconnect

      int block_actor = no_actors + channel_count;
      int send_actor = block_actor + 1;
      int rec_actor = send_actor + 1;
      //add the block actor as a successor of ch_src[i]
      SuccessorNode succB;
      succB.successor_key = block_actor;
      succB.delay = sendingLatency[i];
      succB.min_tok = 0;
      succB.max_tok = 0;
      succB.channel = i;

      //add to boost-msag
      boost_msag_des& curr_graph = *b_msags[msagId[applications->getChannels()[i]->source]];
      src = g.getVertex(applications->getChannels()[i]->source);    //b::vertex(ch_src[i], *b_msags[msagId[ch_src[i]]]);
      dst = g.getVertex(block_actor);  //b::vertex(block_actor, *b_msags[msagId[ch_src[i]]]);
      b::tie(_e, found) = b::add_edge(src, dst, curr_graph);
      b::put(b::edge_weight, curr_graph, _e, sendingLatency[i]);
      b::put(b::edge_weight2, curr_graph, _e, 0);
      //delay-weight for self-loop on block-actor:
      curr_graph = *b_msags[msagId[block_actor]];
      tie(_e, found) = edge(dst, dst, curr_graph);
      b::put(b::edge_weight, curr_graph, _e, sendingLatency[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(applications->getChannels()[i]->source);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(applications->getChannels()[i]->source).push_back(succB);
        }else{      //no entry for ch_src[i] yet
          vector<SuccessorNode> succBv;
          succBv.push_back(succB);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(applications->getChannels()[i]->source, succBv));
        }
      }

      //add ch_src[i] as successor of the block actor, with buffer size as tokens
      SuccessorNode srcCh;
      srcCh.successor_key = applications->getChannels()[i]->source;
      srcCh.delay = wcet[applications->getChannels()[i]->source];
      srcCh.min_tok = sendbufferSz[i];
      srcCh.max_tok = sendbufferSz[i];

      //add to boost-msag
      boost_msag_des& curr_graph1 = *b_msags[msagId[block_actor]];
      src = g.getVertex(block_actor);  //b::vertex(block_actor, *b_msags[msagId[block_actor]]);
      dst = g.getVertex(applications->getChannels()[i]->source);    //b::vertex(ch_src[i], *b_msags[msagId[block_actor]]);
      b::tie(_e, found) = b::add_edge(src, dst, *b_msags[msagId[block_actor]]);
      b::put(b::edge_weight, curr_graph1, _e, wcet[applications->getChannels()[i]->source]);
      b::put(b::edge_weight2, curr_graph1, _e, sendbufferSz[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(block_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(block_actor).push_back(srcCh);
        }else{      //no entry for block_actor yet
          vector<SuccessorNode> srcChv;
          srcChv.push_back(srcCh);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(block_actor, srcChv));
        }
      }
//###
      //add the send actor as a successor of the block actor
      SuccessorNode succS;
      succS.successor_key = send_actor;
      succS.delay = sendingTime[i];
      succS.min_tok = 0;
      succS.max_tok = 0;
      succS.channel = i;

      //add to boost-msag
      boost_msag_des& curr_graph2 = *b_msags[msagId[block_actor]];
      src = g.getVertex(block_actor);  //b::vertex(block_actor, *b_msags[msagId[block_actor]]);
      dst = g.getVertex(send_actor);   //b::vertex(send_actor, *b_msags[msagId[block_actor]]);
      b::tie(_e, found) = b::add_edge(src, dst, curr_graph2);
      b::put(b::edge_weight, curr_graph2, _e, sendingTime[i]);
      b::put(b::edge_weight2, curr_graph2, _e, 0);
      //delay-weight for self-loop on send-actor:
      tie(_e, found) = edge(dst, dst, curr_graph2);
      b::put(b::edge_weight, curr_graph2, _e, sendingTime[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(block_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(block_actor).push_back(succS);
        }else{      //no entry for block_actor yet
          vector<SuccessorNode> succSv;
          succSv.push_back(succS);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(block_actor, succSv));
        }
      }

      //add the block actor as successor of the send actor, with one token (serialization)
      SuccessorNode succBS;
      succBS.successor_key = block_actor;
      succBS.delay = sendingLatency[i];
      succBS.min_tok = 1;
      succBS.max_tok = 1;
      succBS.channel = i;

      //add to boost-msag
      boost_msag_des& curr_graph3 = *b_msags[msagId[send_actor]];
      src = g.getVertex(send_actor);   //b::vertex(send_actor, *b_msags[msagId[send_actor]]);
      dst = g.getVertex(block_actor);  //b::vertex(block_actor, *b_msags[msagId[send_actor]]);
      b::tie(_e, found) = b::add_edge(src, dst, *b_msags[msagId[send_actor]]);
      b::put(b::edge_weight, curr_graph3, _e, sendingLatency[i]);
      b::put(b::edge_weight2, curr_graph3, _e, 1);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(send_actor);
        if(it != msaGraph.end()){ //send actor already has an entry in the map
          msaGraph.at(send_actor).push_back(succBS);
        }else{      //no entry for send_actor yet
          vector<SuccessorNode> succBSv;
          succBSv.push_back(succBS);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(send_actor, succBSv));
        }
      }

      //add receiving actor as successor of the send actor, with potential initial tokens
      SuccessorNode dstCh;
      dstCh.successor_key = rec_actor;
      dstCh.delay = receivingTime[i];
      dstCh.min_tok = applications->getChannels()[i]->initTokens;
      dstCh.max_tok = applications->getChannels()[i]->initTokens;
      dstCh.channel = i;
      dstCh.recOrder = receivingNext[i];

      //add to boost-msag
      boost_msag_des& curr_graph4 = *b_msags[msagId[send_actor]];
      src = g.getVertex(send_actor);   //b::vertex(send_actor, *b_msags[msagId[send_actor]]);
      dst = g.getVertex(rec_actor);    //b::vertex(rec_actor, *b_msags[msagId[send_actor]]);
      b::tie(_e, found) = b::add_edge(src, dst, curr_graph4);
      b::put(b::edge_weight, curr_graph4, _e, receivingTime[i]);
      b::put(b::edge_weight2, curr_graph4, _e, applications->getChannels()[i]->initTokens);
      //delay-weight for self-loop on rec-actor:
      tie(_e, found) = edge(dst, dst, curr_graph4);
      b::put(b::edge_weight, curr_graph4, _e, receivingTime[i]);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(send_actor);
        if(it != msaGraph.end()){    //i already has an entry in the map
          msaGraph.at(send_actor).push_back(dstCh);
        }else{      //no entry for i yet
          vector<SuccessorNode> dstChv;
          dstChv.push_back(dstCh);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(send_actor, dstChv));
        }
      }

      //save the receiving actors for each actor (for next order)
      if(receivingActors[applications->getChannels()[i]->destination] == -1){ //first rec_actor for the dst
        receivingActors[applications->getChannels()[i]->destination] = rec_actor;
      }else{
        int curRec_actor_ch = channelMapping[receivingActors[applications->getChannels()[i]->destination] - no_actors];
        //if(receivingNext[curRec_actor_ch].assigned())
        {
          if(receivingNext[curRec_actor_ch] < (int) no_channels){
            if(applications->getChannels()[receivingNext[curRec_actor_ch]]->destination != applications->getChannels()[i]->destination){ //last rec_actor for this dst
              receivingActors[applications->getChannels()[i]->destination] = rec_actor;
            } //else
          }else{ //last rec_actor for this dst
            receivingActors[applications->getChannels()[i]->destination] = rec_actor;
          }
        }
        //if(receivingNext[channelMapping[rec_actor - no_actors]].assigned())
        {
          if(receivingNext[channelMapping[rec_actor - no_actors]] == receivingActors[applications->getChannels()[i]->destination]){
            receivingActors[applications->getChannels()[i]->destination] = rec_actor;
          }
        }
      }

      //add the send actor as a successor node of the receiving actor, with rec. buffer size - initial tokens
      SuccessorNode succRec;
      succRec.successor_key = send_actor;
      succRec.delay = sendingTime[i];
      succRec.min_tok = recbufferSz[i] - applications->getChannels()[i]->initTokens;
      succRec.max_tok = recbufferSz[i] - applications->getChannels()[i]->initTokens;
      succRec.channel = i;

      //add to boost-msag
      boost_msag_des& curr_graph5 = *b_msags[msagId[rec_actor]];
      src = g.getVertex(rec_actor);   //b::vertex(rec_actor, *b_msags[msagId[rec_actor]]);
      dst = g.getVertex(send_actor);   //b::vertex(send_actor, *b_msags[msagId[rec_actor]]);
      b::tie(_e, found) = b::add_edge(src, dst, curr_graph5);
      b::put(b::edge_weight, curr_graph5, _e, sendingTime[i]);
      b::put(b::edge_weight2, curr_graph5, _e, recbufferSz[i] - applications->getChannels()[i]->initTokens);

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(rec_actor);
        if(it != msaGraph.end()){ //i already has an entry in the map
          msaGraph.at(rec_actor).push_back(succRec);
        }else{ //no entry for i yet
          vector<SuccessorNode> succRecv;
          succRecv.push_back(succRec);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(rec_actor, succRecv));
        }
      }

      channel_count += 3;
    }else if(sendingTime[i] == 0){ //Step 1b: add all edges from G to the MSAG
      if((applications->getChannels()[i]->initTokens > 0)){
        //ch_src[i] -> ch_dst[i]: add channel destination as successor node of the channel source
        SuccessorNode _dst;
        _dst.successor_key = applications->getChannels()[i]->destination;
        _dst.delay = wcet[applications->getChannels()[i]->destination];
        _dst.min_tok = applications->getChannels()[i]->initTokens;
        _dst.max_tok = applications->getChannels()[i]->initTokens;
        _dst.channel = i;

        //add to boost-msag
        boost_msag_des& curr_graph5 = *b_msags[msagId[applications->getChannels()[i]->source]];
        src = g.getVertex(applications->getChannels()[i]->source);   //b::vertex(ch_src[i], *b_msags[msagId[ch_src[i]]]);
        dst = g.getVertex(applications->getChannels()[i]->destination);   //b::vertex(ch_dst[i], *b_msags[msagId[ch_src[i]]]);
        b::tie(_e, found) = b::add_edge(src, dst, curr_graph5);
        b::put(b::edge_weight, curr_graph5, _e, wcet[applications->getChannels()[i]->destination]);
        b::put(b::edge_weight2, curr_graph5, _e, applications->getChannels()[i]->initTokens);

        n_msagChannels++;
        if(printDebug){
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(applications->getChannels()[i]->source);
          if(it != msaGraph.end()){ //i already has an entry in the map
            msaGraph.at(applications->getChannels()[i]->source).push_back(_dst);
          }else{ //no entry for i yet
            vector<SuccessorNode> dstv;
            dstv.push_back(_dst);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(applications->getChannels()[i]->source, dstv));
          }
        }
      }
    }
  }
  

  //put sendNext relations into the MSAG
  for(unsigned int i = 1; i < channelMapping.size(); i += 3){ //for all sending actors
    bool continues = true;
    bool nextFound = false;
    int nextCh;
    int x = channelMapping[i];
    int tokens = 0; //is channel to add a cycle-closing back-edge?
    while(!nextFound && continues){
      //if(sendingNext[x].assigned())
      {
        nextCh = sendingNext[x];
        if(nextCh >= (int) no_channels){ //end of chain found
          if(nextCh > (int) no_channels){
            nextCh = no_channels + ((nextCh - no_channels - 1) % no_processors);
          }else{
            nextCh = no_channels + no_processors - 1;
          }
          tokens = 1;
          //if(sendingNext[nextCh].assigned())
          {
            nextCh = sendingNext[nextCh];
            if(sendingTime[nextCh] > 0){
              nextFound = true;
            }else{
              x = nextCh;
            }
          }/*else{
            continues = false;
          }*/
        }else{ //not end of chain (nextCh < n_channels)
          if(sendingTime[nextCh] > 0){
            nextFound = true;
            if(tokens != 1)
              tokens = 0;
          }else{
            x = nextCh; //nextCh is not on interconnect. Continue with nextSend[nextCh].
          }
        }
      }/*else{
        continues = false;
      }*/
    }
    if(nextFound){
      //add send_actor of channel i -> block_actor of nextCh
      if(channelMapping[i] != nextCh){ //if found successor is not the channel's own block_actor (then it is already in the graph)
        int block_actor = getBlockActor(nextCh);

        //cout << "Next channel (send_actor of channel " << i << "): " << nextCh << endl;

        SuccessorNode succBS;
        succBS.successor_key = block_actor;
        succBS.delay = sendingLatency[nextCh];
        succBS.min_tok = tokens;
        succBS.max_tok = tokens;
        succBS.channel = nextCh;

        //add to boost-msag
        boost_msag_des& curr_graph6 = *b_msags[msagId[block_actor]];
        src = g.getVertex(i + no_actors);   //b::vertex(i + no_actors, *b_msags[msagId[block_actor]]);
        dst = g.getVertex(block_actor);    //b::vertex(block_actor, *b_msags[msagId[block_actor]]);
        b::tie(_e, found) = b::add_edge(src, dst, curr_graph6);
        b::put(b::edge_weight, curr_graph6, _e, sendingLatency[nextCh]);
        b::put(b::edge_weight2, curr_graph6, _e, tokens);

        n_msagChannels++;
        if(printDebug){
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i + no_actors);
          if(it != msaGraph.end()){ //send actor already has an entry in the map
            msaGraph.at(i + no_actors).push_back(succBS);
          }else{ //no entry for send_actor i yet
            vector<SuccessorNode> succBSv;
            succBSv.push_back(succBS);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(i + no_actors, succBSv));
          }
        }
      }
    }
  }

  //put recNext relations into the MSAG
  for(unsigned int i = 2; i < channelMapping.size(); i += 3){ //for all receiving actors
    bool nextFound = false;
    int nextCh;
    int x = channelMapping[i];

    //cout << "looking for recNext, channel " << i << endl;

    while(!nextFound){
      //if(receivingNext[x].assigned())
      {
        nextCh = receivingNext[x];
        if(nextCh >= (int) no_channels){ //end of chain found
          nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
          nextFound = true;
        }else{ //not end of chain (nextCh < n_channels)
          if(applications->getChannels()[nextCh]->destination != applications->getChannels()[channelMapping[i]]->destination){ //next rec actor belongs to other dst
            nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
            nextFound = true;
          }else{ //same dst
            if(sendingTime[nextCh] > 0){
              nextFound = true;
            }else{
              x = nextCh; //nextCh is not on interconnect. Continue with nextSend[nextCh].
            }
          }
        }
      }/*else{
        nextCh = -1; //nextCh = ch_dst[channelMapping[i]];
        nextFound = true;
      }*/
    }

    //cout << "  found " << nextCh;

    SuccessorNode succRec;
    succRec.successor_key = nextCh == -1 ? applications->getChannels()[channelMapping[i]]->destination : getRecActor(nextCh);
    succRec.delay = nextCh == -1 ? wcet[applications->getChannels()[channelMapping[i]]->destination] : receivingTime[nextCh];
    succRec.min_tok = 0;
    succRec.max_tok = 0;
    if(nextCh != -1)
      succRec.channel = nextCh;

    //cout << " ( "<< succRec.successor_key <<")" << endl;

    //add to boost-msag
    int tmp = i + no_actors;
    boost_msag_des& curr_graph7 = *b_msags[msagId[tmp]];
    src = g.getVertex(tmp);            //b::vertex(tmp, *b_msags[msagId[tmp]]);
    dst = g.getVertex(nextCh == -1 ? applications->getChannels()[channelMapping[i]]->destination : getRecActor(nextCh)); //b::vertex(nextCh == -1 ? ch_dst[channelMapping[i]] : getRecActor(nextCh),*b_msags[msagId[tmp]]);
    b::tie(_e, found) = b::add_edge(src, dst, curr_graph7);
    b::put(b::edge_weight, curr_graph7, _e, nextCh == -1 ? wcet[applications->getChannels()[channelMapping[i]]->destination] : receivingTime[nextCh]);
    b::put(b::edge_weight2, curr_graph7, _e, 0);

    n_msagChannels++;
    if(printDebug){
      unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i + no_actors);
      if(it != msaGraph.end()){ //send actor already has an entry in the map
        msaGraph.at(i + no_actors).push_back(succRec);
      }else{ //no entry for send_actor i yet
        vector<SuccessorNode> succRecv;
        succRecv.push_back(succRec);
        msaGraph.insert(pair<int, vector<SuccessorNode>>(i + no_actors, succRecv));
      }
    }
  }

  for(size_t i = 0; i < no_actors; i++){
    //Step 2
    if(next[i] < (int) no_actors){ //if next[i] is decided, the forward edge goes from i to next[i]
      int nextActor = next[i];

      //check whether nextActor has preceding rec_actor
      SuccessorNode nextA;
      if(receivingActors[nextActor] == -1){
        //add edge i -> nextActor
        nextA.successor_key = nextActor;
        nextA.delay = wcet[nextActor];
        nextA.min_tok = 0;
        nextA.max_tok = 0;

        //add to boost-msag
        boost_msag_des& curr_graph8 = *b_msags[msagId[i]];
        src = g.getVertex(i);         //b::vertex(i, *b_msags[msagId[i]]);
        dst = g.getVertex(nextActor); //b::vertex(nextActor, *b_msags[msagId[i]]);
        b::tie(_e, found) = b::add_edge(src, dst, curr_graph8);
        b::put(b::edge_weight, curr_graph8, _e, wcet[nextActor]);
        b::put(b::edge_weight2, curr_graph8, _e, 0);
      }else{
        //add edge i -> receivingActor[nextActor]
        nextA.successor_key = receivingActors[nextActor];
        nextA.delay = receivingTime[channelMapping[receivingActors[nextActor] - no_actors]];
        nextA.min_tok = 0;
        nextA.max_tok = 0;
        nextA.channel = channelMapping[receivingActors[nextActor] - no_actors];

        //add to boost-msag
        boost_msag_des& curr_graph9 = *b_msags[msagId[i]];
        src = g.getVertex(i);                          //b::vertex(i, *b_msags[msagId[i]]);
        dst = g.getVertex(receivingActors[nextActor]); //b::vertex(receivingActors[nextActor], *b_msags[msagId[i]]);
        b::tie(_e, found) = b::add_edge(src, dst, curr_graph9);
        b::put(b::edge_weight, curr_graph9, _e, receivingTime[channelMapping[receivingActors[nextActor] - no_actors]]);
        b::put(b::edge_weight2, curr_graph9, _e, 0);
      }

      n_msagChannels++;
      if(printDebug){
        unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i);
        if(it != msaGraph.end()){ //i already has an entry in the map
          msaGraph.at(i).push_back(nextA);
        }else{ //no entry for i yet
          vector<SuccessorNode> nextAv;
          nextAv.push_back(nextA);
          msaGraph.insert(pair<int, vector<SuccessorNode>>(i, nextAv));
        }
      }

    }else if(next[i] >= (int) no_actors){ //next[i]>=no_actors
    //Step 3: add cycle-closing edge on each proc
      int firstActor = next[i];
      if(firstActor > (int) no_actors){
        firstActor = no_actors + ((firstActor - no_actors - 1) % no_processors);
      }else{
        firstActor = no_actors + no_processors - 1;
      }
      //if(next[firstActor].assigned())
      {
        firstActor = next[firstActor];

        //check whether firstActor has preceding rec_actor
        SuccessorNode first;
        if(receivingActors[firstActor] == -1){
          //add edge i -> firstActor
          first.successor_key = firstActor;
          first.delay = wcet[firstActor];
          first.min_tok = 1;
          first.max_tok = 1;

          //add to boost-msag
          boost_msag_des& curr_graph10 = *b_msags[msagId[i]];
          src = g.getVertex(i);          //b::vertex(i, *b_msags[msagId[i]]);
          dst = g.getVertex(firstActor); //b::vertex(firstActor, *b_msags[msagId[i]]);
          b::tie(_e, found) = b::add_edge(src, dst, curr_graph10);
          b::put(b::edge_weight, curr_graph10, _e, wcet[firstActor]);
          b::put(b::edge_weight2, curr_graph10, _e, 1);
        }else{
          //add edge i -> receivingActor[firstActor]
          first.successor_key = receivingActors[firstActor];
          first.delay = receivingTime[channelMapping[receivingActors[firstActor] - no_actors]];
          first.min_tok = 1;
          first.max_tok = 1;
          first.channel = channelMapping[receivingActors[firstActor] - no_actors];

          //add to boost-msag
          boost_msag_des& curr_graph11 = *b_msags[msagId[i]];
          src = g.getVertex(i);                           //b::vertex(i, *b_msags[msagId[i]]);
          dst = g.getVertex(receivingActors[firstActor]); //b::vertex(receivingActors[firstActor], *b_msags[msagId[i]]);
          b::tie(_e, found) = b::add_edge(src, dst, curr_graph11);
          b::put(b::edge_weight, curr_graph11, _e, receivingTime[channelMapping[receivingActors[firstActor] - no_actors]]);
          b::put(b::edge_weight2, curr_graph11, _e, 1);
        }

        n_msagChannels++;
        if(printDebug){
          unordered_map<int, vector<SuccessorNode>>::const_iterator it = msaGraph.find(i);
          if(it != msaGraph.end()){ //i already has an entry in the map
            msaGraph.at(i).push_back(first);
          }else{    //no entry for i yet
            vector<SuccessorNode> firstv;
            firstv.push_back(first);
            msaGraph.insert(pair<int, vector<SuccessorNode>>(i, firstv));
          }
        }
      }
    }
  }

  if(printDebug){
    //printThroughputGraphAsDot(".");
  }
}

void Design::print_vector(const vector<int> input)
{
    for(auto i : input)
        cout << i << ",";    
    cout << "}\n";    
}

void Design::calc_energy()
{
    vector<int> sum_wcet_proc(no_processors, 0);
    for(size_t i=0;i<no_actors;i++)
    {
        auto proc_id = proc_mappings[i];
        sum_wcet_proc[proc_id] += wcet[i];
    }
    vector<int> utilizations(no_processors, 0);    
    energy = 0; 
    /**
     * Since applications mapped to the same processor have same periods,
     * we can derive the processor periods as follows:
     */ 
    vector<int> proc_periods(no_processors, 0);
    for(size_t i=0;i<no_actors;i++)
    {
        proc_periods[proc_mappings[i]] = periods[applications->getSDFGraph(i)];
    }
    for(size_t i=0;i<no_processors;i++)
    {
        if(proc_periods[i] > 0)
            utilizations[i] =  ceil(((float)mapping->max_utilization*sum_wcet_proc[i])/proc_periods[i]);  
             
        energy += utilizations[i] * mapping->getPlatform()->getPowerCons(i)[proc_modes[i]];     
    }    
}
vector<int> Design::get_periods()
{
    calc_periods();
    /*if(periods[0] < 0)
    {
        cout << endl << endl << *this;
        THROW_EXCEPTION(RuntimeException, "deadlock in the schedule" );
    } 
    */    
    return periods;
}
int Design::get_energy()
{
    calc_energy();
    return energy;
}
std::ostream& operator<< (std::ostream &out, const Design &des)
{
    out << "proc_mappings:";
    const string sep = ",";
    for(auto m: des.proc_mappings)
        out << m << sep;
    out << endl;
    
    out << "proc_modes:";
    for(auto m: des.proc_modes)
        out << m << sep;
    out << endl;
        
    out << "next:";
    for(auto n: des.next)
        out << n << sep;
    out << endl;
    
    out << "sendingNext:";
    for(auto s: des.sendingNext)
        out << s << sep;
    out << endl;
    
    out << "receivingNext:";
    for(auto r: des.receivingNext)
        out << r << sep;
    out << endl;
    
    out << "tdmaAlloc:";
    for(auto m: des.tdmaAlloc)
        out << m << sep;
    out << endl;
    
    out << "sendingLatency:";
    for(auto l: des.sendingLatency)
        out << l << sep;
    out << endl;
    
    out << "receivingTime:";
    for(auto m: des.receivingTime)
        out << m << sep;
    out << endl;
    
    out << "sendbufferSz:";
    for(auto b: des.sendbufferSz)
        out << b << sep;
    out << endl;
    
    out << "recbufferSz:";
    for(auto b: des.recbufferSz)
        out << b << sep;
    out << endl;
    
    out << "memCons:";
    for(auto m: des.memCons)
        out << m << sep;
    out << endl;
             
    return out;
}
