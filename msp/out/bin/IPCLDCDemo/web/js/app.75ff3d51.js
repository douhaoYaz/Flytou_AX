(function(){var e={9157:function(e,t,n){"use strict";var r=n(144),o=function(){var e=this,t=e.$createElement,n=e._self._c||t;return n("div",{attrs:{id:"app"}},[n("router-view")],1)},u=[],a={name:"app",created(){var e=window.location.href;e=e.substring(0,e.indexOf("8081/")+5)+"action",this.$http.defaults.baseURL=e,console.log(e)}},i=a,s=n(3736),f=(0,s.Z)(i,o,u,!1,null,null,null),c=f.exports,d=n(8345);const l=()=>n.e(497).then(n.bind(n,8434)),p=()=>n.e(726).then(n.bind(n,339)),h=()=>n.e(559).then(n.bind(n,8139)),m=()=>n.e(348).then(n.bind(n,5562)),g=()=>n.e(348).then(n.bind(n,8086)),v=()=>n.e(348).then(n.bind(n,2887)),b=()=>n.e(348).then(n.bind(n,1235)),y=()=>n.e(348).then(n.bind(n,8763)),w=()=>n.e(348).then(n.bind(n,2474)),k=()=>n.e(348).then(n.bind(n,1127)),x=()=>n.e(348).then(n.bind(n,8537));r["default"].use(d.Z);const O=[{path:"/",redirect:"/login"},{path:"/login",component:l},{path:"/home",component:p,redirect:"/preview",children:[{path:"/preview",component:h},{path:"/setting",component:m,redirect:"/setting/system",children:[{path:"/setting/system",component:g},{path:"/setting/camera",component:v},{path:"/setting/ai",component:b},{path:"/setting/overlay",component:y},{path:"/setting/video",component:w},{path:"/setting/storage",component:k},{path:"/setting/playback",component:x}]}]}],C=new d.Z({routes:O});C.beforeEach(((e,t,n)=>{if(document.title="IPCLDCDemo","/login"===e.path)return n();const r=window.sessionStorage.getItem("token");if(!r)return n("/login");n()}));const S=d.Z.prototype.push;d.Z.prototype.push=function(e){return S.call(this,e).catch((e=>e))};var E=C,j=n(9669),A=n.n(j),T=n(6905),$=n(1337),_=n.n($),P=n(1955),L=n.n(P),N=n(6177),Z=n.n(N),F=n(5034),D=n.n(F),I=n(3364),q=n.n(I),B=n(5652),M=n.n(B),K=n(5517),U=n.n(K),z=n(4242),H=n.n(z),R=n(5614),G=n.n(R),J=n(1530),Q=n.n(J),V=n(905),W=n.n(V),X=n(4243),Y=n.n(X),ee=n(3492),te=n.n(ee),ne=n(7186),re=n.n(ne),oe=n(2618),ue=n.n(oe),ae=n(7987),ie=n.n(ae),se=n(7608),fe=n.n(se),ce=n(8970),de=n.n(ce),le=n(4561),pe=n.n(le),he=n(8331),me=n.n(he),ge=n(2173),ve=n.n(ge),be=n(3229),ye=n.n(be),we=n(7099),ke=n.n(we),xe=n(1119),Oe=n.n(xe),Ce=n(642),Se=n.n(Ce),Ee=n(4433),je=n.n(Ee),Ae=n(7092),Te=n.n(Ae),$e=n(2182),_e=n.n($e),Pe=n(5303),Le=n.n(Pe),Ne=n(4947),Ze=n.n(Ne),Fe=n(7626),De=n.n(Fe),Ie=n(2626),qe=n.n(Ie),Be=n(32),Me=n.n(Be),Ke=n(4223),Ue=n.n(Ke),ze=n(6426),He=n.n(ze),Re=n(6473),Ge=n.n(Re);r["default"].prototype.$message=function(e){Ge()(e)},r["default"].prototype.$message=function(e){return Ge()({message:e,duration:2e3})},r["default"].prototype.$message.success=function(e){return Ge().success({message:e,duration:1e3})},r["default"].prototype.$message.warning=function(e){return Ge().warning({message:e,duration:1500})},r["default"].prototype.$message.error=function(e){return Ge().error({message:e,duration:3e3})},r["default"].use(He()),r["default"].use(Ue()),r["default"].use(Me()),r["default"].use(qe()),r["default"].use(De()),r["default"].use(Ze()),r["default"].use(Le()),r["default"].use(_e()),r["default"].use(Te()),r["default"].use(je()),r["default"].use(Se()),r["default"].use(Oe()),r["default"].use(ke()),r["default"].use(ye()),r["default"].use(ve()),r["default"].use(me()),r["default"].use(pe()),r["default"].use(de()),r["default"].use(fe()),r["default"].use(ie()),r["default"].use(ue()),r["default"].use(re()),r["default"].use(te()),r["default"].use(Y()),r["default"].use(W()),r["default"].use(Q()),r["default"].use(G()),r["default"].use(H()),r["default"].use(U()),r["default"].use(M()),r["default"].use(q()),r["default"].use(D()),r["default"].use(Z()),r["default"].use(L()),r["default"].use(_());var Je=n(8041),Qe=n.n(Je);const Ve=new(Qe())({maxSockets:100,maxFreeSockets:10,timeout:6e4,freeSocketKeepAliveTimeout:3e4});r["default"].prototype.$video=T.Z,A().interceptors.request.use((e=>(e.headers.Authorization=window.sessionStorage.getItem("token"),e))),A().interceptors.response.use((e=>e)),A().defaults.httpAgent=Ve,r["default"].prototype.$http=A(),r["default"].config.productionTip=!1,new r["default"]({router:E,render:e=>e(c)}).$mount("#app")},5893:function(){}},t={};function n(r){var o=t[r];if(void 0!==o)return o.exports;var u=t[r]={exports:{}};return e[r].call(u.exports,u,u.exports,n),u.exports}n.m=e,function(){var e=[];n.O=function(t,r,o,u){if(!r){var a=1/0;for(c=0;c<e.length;c++){r=e[c][0],o=e[c][1],u=e[c][2];for(var i=!0,s=0;s<r.length;s++)(!1&u||a>=u)&&Object.keys(n.O).every((function(e){return n.O[e](r[s])}))?r.splice(s--,1):(i=!1,u<a&&(a=u));if(i){e.splice(c--,1);var f=o();void 0!==f&&(t=f)}}return t}u=u||0;for(var c=e.length;c>0&&e[c-1][2]>u;c--)e[c]=e[c-1];e[c]=[r,o,u]}}(),function(){n.n=function(e){var t=e&&e.__esModule?function(){return e["default"]}:function(){return e};return n.d(t,{a:t}),t}}(),function(){n.d=function(e,t){for(var r in t)n.o(t,r)&&!n.o(e,r)&&Object.defineProperty(e,r,{enumerable:!0,get:t[r]})}}(),function(){n.f={},n.e=function(e){return Promise.all(Object.keys(n.f).reduce((function(t,r){return n.f[r](e,t),t}),[]))}}(),function(){n.u=function(e){return"js/"+{348:"ax3",497:"ax0",559:"ax2",726:"ax1"}[e]+"."+{348:"472f0d40",497:"ba602944",559:"4c0e21b2",726:"dc13acd2"}[e]+".js"}}(),function(){n.miniCssF=function(e){return"css/"+{348:"ax3",497:"ax0",559:"ax2",726:"ax1"}[e]+"."+{348:"51f16d09",497:"51a781e9",559:"e3d70280",726:"49967c54"}[e]+".css"}}(),function(){n.g=function(){if("object"===typeof globalThis)return globalThis;try{return this||new Function("return this")()}catch(e){if("object"===typeof window)return window}}()}(),function(){n.o=function(e,t){return Object.prototype.hasOwnProperty.call(e,t)}}(),function(){var e={},t="webapp:";n.l=function(r,o,u,a){if(e[r])e[r].push(o);else{var i,s;if(void 0!==u)for(var f=document.getElementsByTagName("script"),c=0;c<f.length;c++){var d=f[c];if(d.getAttribute("src")==r||d.getAttribute("data-webpack")==t+u){i=d;break}}i||(s=!0,i=document.createElement("script"),i.charset="utf-8",i.timeout=120,n.nc&&i.setAttribute("nonce",n.nc),i.setAttribute("data-webpack",t+u),i.src=r),e[r]=[o];var l=function(t,n){i.onerror=i.onload=null,clearTimeout(p);var o=e[r];if(delete e[r],i.parentNode&&i.parentNode.removeChild(i),o&&o.forEach((function(e){return e(n)})),t)return t(n)},p=setTimeout(l.bind(null,void 0,{type:"timeout",target:i}),12e4);i.onerror=l.bind(null,i.onerror),i.onload=l.bind(null,i.onload),s&&document.head.appendChild(i)}}}(),function(){n.r=function(e){"undefined"!==typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(e,"__esModule",{value:!0})}}(),function(){n.p="/"}(),function(){var e=function(e,t,n,r){var o=document.createElement("link");o.rel="stylesheet",o.type="text/css";var u=function(u){if(o.onerror=o.onload=null,"load"===u.type)n();else{var a=u&&("load"===u.type?"missing":u.type),i=u&&u.target&&u.target.href||t,s=new Error("Loading CSS chunk "+e+" failed.\n("+i+")");s.code="CSS_CHUNK_LOAD_FAILED",s.type=a,s.request=i,o.parentNode.removeChild(o),r(s)}};return o.onerror=o.onload=u,o.href=t,document.head.appendChild(o),o},t=function(e,t){for(var n=document.getElementsByTagName("link"),r=0;r<n.length;r++){var o=n[r],u=o.getAttribute("data-href")||o.getAttribute("href");if("stylesheet"===o.rel&&(u===e||u===t))return o}var a=document.getElementsByTagName("style");for(r=0;r<a.length;r++){o=a[r],u=o.getAttribute("data-href");if(u===e||u===t)return o}},r=function(r){return new Promise((function(o,u){var a=n.miniCssF(r),i=n.p+a;if(t(a,i))return o();e(r,i,o,u)}))},o={143:0};n.f.miniCss=function(e,t){var n={348:1,497:1,559:1,726:1};o[e]?t.push(o[e]):0!==o[e]&&n[e]&&t.push(o[e]=r(e).then((function(){o[e]=0}),(function(t){throw delete o[e],t})))}}(),function(){var e={143:0};n.f.j=function(t,r){var o=n.o(e,t)?e[t]:void 0;if(0!==o)if(o)r.push(o[2]);else{var u=new Promise((function(n,r){o=e[t]=[n,r]}));r.push(o[2]=u);var a=n.p+n.u(t),i=new Error,s=function(r){if(n.o(e,t)&&(o=e[t],0!==o&&(e[t]=void 0),o)){var u=r&&("load"===r.type?"missing":r.type),a=r&&r.target&&r.target.src;i.message="Loading chunk "+t+" failed.\n("+u+": "+a+")",i.name="ChunkLoadError",i.type=u,i.request=a,o[1](i)}};n.l(a,s,"chunk-"+t,t)}},n.O.j=function(t){return 0===e[t]};var t=function(t,r){var o,u,a=r[0],i=r[1],s=r[2],f=0;if(a.some((function(t){return 0!==e[t]}))){for(o in i)n.o(i,o)&&(n.m[o]=i[o]);if(s)var c=s(n)}for(t&&t(r);f<a.length;f++)u=a[f],n.o(e,u)&&e[u]&&e[u][0](),e[u]=0;return n.O(c)},r=self["webpackChunkwebapp"]=self["webpackChunkwebapp"]||[];r.forEach(t.bind(null,0)),r.push=t.bind(null,r.push.bind(r))}();var r=n.O(void 0,[998],(function(){return n(9157)}));r=n.O(r)})();