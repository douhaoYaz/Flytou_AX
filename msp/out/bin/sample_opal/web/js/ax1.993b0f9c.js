(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["ax1"],{"1da1":function(t,e,r){"use strict";r.d(e,"a",(function(){return o}));r("d3b7");function n(t,e,r,n,o,i,a){try{var c=t[i](a),s=c.value}catch(u){return void r(u)}c.done?e(s):Promise.resolve(s).then(n,o)}function o(t){return function(){var e=this,r=arguments;return new Promise((function(o,i){var a=t.apply(e,r);function c(t){n(a,o,i,c,s,"next",t)}function s(t){n(a,o,i,c,s,"throw",t)}c(void 0)}))}}},"57da":function(t,e,r){"use strict";r.r(e);var n=function(){var t=this,e=t.$createElement,n=t._self._c||e;return n("div",{staticClass:"home"},[n("el-container",{staticClass:"home-container"},[n("el-header",[n("div",{staticClass:"header_logo"},[n("img",{attrs:{alt:"logo",src:r("cf05")}})]),n("div",{staticClass:"header_ver"},[n("span",[t._v("OpalDemo SDK: "+t._s(this.sdkVersion))])]),n("div",[n("el-divider",{attrs:{direction:"vertical"}})],1),n("div",{staticClass:"header_nav"},[n("el-menu",{attrs:{router:"","default-active":t.activePath,mode:"horizontal","background-color":"#4c4c4c","text-color":"#fff","active-text-color":"#fff"},on:{select:t.handleSelect}},[n("el-menu-item",{attrs:{index:"/preview"},on:{click:function(e){return t.saveNavState("/preview")}}},[t._v("预览")]),n("el-menu-item",{attrs:{index:"/setting"},on:{click:function(e){return t.saveNavState("/setting")}}},[t._v("配置")])],1)],1),n("div",{staticClass:"header_right",attrs:{align:"right"}},[n("i",{staticClass:"el-icon-s-custom"}),n("span",[t._v("admin")]),n("el-tooltip",{attrs:{effect:"dark",content:"退出登录",placement:"bottom-end",enterable:!1}},[n("el-button",{attrs:{type:"text",icon:"el-icon-switch-button"},on:{click:t.logout}})],1)],1)]),n("el-main",[n("router-view")],1)],1)],1)},o=[],i=r("1da1"),a=(r("96cf"),{data:function(){return{appVersion:"--",sdkVersion:"--",activePath:"/preview"}},created:function(){var t=window.location.href;t=t.substring(0,t.indexOf("8085/")+5)+"action",this.$http.defaults.baseURL=t,this.getInfo();var e=window.sessionStorage.getItem("homeActivePath");e&&(this.activePath=e),console.log("home:",this.activePath),this.$router.path!==this.activePath&&this.$router.push(this.activePath)},methods:{getInfo:function(){var t=this;return Object(i["a"])(regeneratorRuntime.mark((function e(){var r,n;return regeneratorRuntime.wrap((function(e){while(1)switch(e.prev=e.next){case 0:return e.prev=0,e.next=3,t.$http.get("setting/system");case 3:r=e.sent,n=r.data,console.log("home page, system get return: ",n),200===n.meta.status&&(t.appVersion=n.data.appVersion,t.sdkVersion=n.data.sdkVersion),e.next=12;break;case 9:e.prev=9,e.t0=e["catch"](0),t.$message.error("获取信息失败");case 12:case"end":return e.stop()}}),e,null,[[0,9]])})))()},handleSelect:function(t,e){console.log(t,e)},logout:function(){window.sessionStorage.clear(),this.$router.push("/login")},saveNavState:function(t){window.sessionStorage.setItem("homeActivePath",t)}}}),c=a,s=(r("d45b"),r("2877")),u=Object(s["a"])(c,n,o,!1,null,"42396aa4",null);e["default"]=u.exports},"96cf":function(t,e){!function(e){"use strict";var r,n=Object.prototype,o=n.hasOwnProperty,i="function"===typeof Symbol?Symbol:{},a=i.iterator||"@@iterator",c=i.asyncIterator||"@@asyncIterator",s=i.toStringTag||"@@toStringTag",u="object"===typeof t,h=e.regeneratorRuntime;if(h)u&&(t.exports=h);else{h=e.regeneratorRuntime=u?t.exports:{},h.wrap=x;var l="suspendedStart",f="suspendedYield",p="executing",v="completed",d={},g={};g[a]=function(){return this};var y=Object.getPrototypeOf,m=y&&y(y(C([])));m&&m!==n&&o.call(m,a)&&(g=m);var w=E.prototype=L.prototype=Object.create(g);_.prototype=w.constructor=E,E.constructor=_,E[s]=_.displayName="GeneratorFunction",h.isGeneratorFunction=function(t){var e="function"===typeof t&&t.constructor;return!!e&&(e===_||"GeneratorFunction"===(e.displayName||e.name))},h.mark=function(t){return Object.setPrototypeOf?Object.setPrototypeOf(t,E):(t.__proto__=E,s in t||(t[s]="GeneratorFunction")),t.prototype=Object.create(w),t},h.awrap=function(t){return{__await:t}},k(P.prototype),P.prototype[c]=function(){return this},h.AsyncIterator=P,h.async=function(t,e,r,n){var o=new P(x(t,e,r,n));return h.isGeneratorFunction(e)?o:o.next().then((function(t){return t.done?t.value:o.next()}))},k(w),w[s]="Generator",w[a]=function(){return this},w.toString=function(){return"[object Generator]"},h.keys=function(t){var e=[];for(var r in t)e.push(r);return e.reverse(),function r(){while(e.length){var n=e.pop();if(n in t)return r.value=n,r.done=!1,r}return r.done=!0,r}},h.values=C,G.prototype={constructor:G,reset:function(t){if(this.prev=0,this.next=0,this.sent=this._sent=r,this.done=!1,this.delegate=null,this.method="next",this.arg=r,this.tryEntries.forEach(N),!t)for(var e in this)"t"===e.charAt(0)&&o.call(this,e)&&!isNaN(+e.slice(1))&&(this[e]=r)},stop:function(){this.done=!0;var t=this.tryEntries[0],e=t.completion;if("throw"===e.type)throw e.arg;return this.rval},dispatchException:function(t){if(this.done)throw t;var e=this;function n(n,o){return c.type="throw",c.arg=t,e.next=n,o&&(e.method="next",e.arg=r),!!o}for(var i=this.tryEntries.length-1;i>=0;--i){var a=this.tryEntries[i],c=a.completion;if("root"===a.tryLoc)return n("end");if(a.tryLoc<=this.prev){var s=o.call(a,"catchLoc"),u=o.call(a,"finallyLoc");if(s&&u){if(this.prev<a.catchLoc)return n(a.catchLoc,!0);if(this.prev<a.finallyLoc)return n(a.finallyLoc)}else if(s){if(this.prev<a.catchLoc)return n(a.catchLoc,!0)}else{if(!u)throw new Error("try statement without catch or finally");if(this.prev<a.finallyLoc)return n(a.finallyLoc)}}}},abrupt:function(t,e){for(var r=this.tryEntries.length-1;r>=0;--r){var n=this.tryEntries[r];if(n.tryLoc<=this.prev&&o.call(n,"finallyLoc")&&this.prev<n.finallyLoc){var i=n;break}}i&&("break"===t||"continue"===t)&&i.tryLoc<=e&&e<=i.finallyLoc&&(i=null);var a=i?i.completion:{};return a.type=t,a.arg=e,i?(this.method="next",this.next=i.finallyLoc,d):this.complete(a)},complete:function(t,e){if("throw"===t.type)throw t.arg;return"break"===t.type||"continue"===t.type?this.next=t.arg:"return"===t.type?(this.rval=this.arg=t.arg,this.method="return",this.next="end"):"normal"===t.type&&e&&(this.next=e),d},finish:function(t){for(var e=this.tryEntries.length-1;e>=0;--e){var r=this.tryEntries[e];if(r.finallyLoc===t)return this.complete(r.completion,r.afterLoc),N(r),d}},catch:function(t){for(var e=this.tryEntries.length-1;e>=0;--e){var r=this.tryEntries[e];if(r.tryLoc===t){var n=r.completion;if("throw"===n.type){var o=n.arg;N(r)}return o}}throw new Error("illegal catch attempt")},delegateYield:function(t,e,n){return this.delegate={iterator:C(t),resultName:e,nextLoc:n},"next"===this.method&&(this.arg=r),d}}}function x(t,e,r,n){var o=e&&e.prototype instanceof L?e:L,i=Object.create(o.prototype),a=new G(n||[]);return i._invoke=O(t,r,a),i}function b(t,e,r){try{return{type:"normal",arg:t.call(e,r)}}catch(n){return{type:"throw",arg:n}}}function L(){}function _(){}function E(){}function k(t){["next","throw","return"].forEach((function(e){t[e]=function(t){return this._invoke(e,t)}}))}function P(t){function e(r,n,i,a){var c=b(t[r],t,n);if("throw"!==c.type){var s=c.arg,u=s.value;return u&&"object"===typeof u&&o.call(u,"__await")?Promise.resolve(u.__await).then((function(t){e("next",t,i,a)}),(function(t){e("throw",t,i,a)})):Promise.resolve(u).then((function(t){s.value=t,i(s)}),a)}a(c.arg)}var r;function n(t,n){function o(){return new Promise((function(r,o){e(t,n,r,o)}))}return r=r?r.then(o,o):o()}this._invoke=n}function O(t,e,r){var n=l;return function(o,i){if(n===p)throw new Error("Generator is already running");if(n===v){if("throw"===o)throw i;return I()}r.method=o,r.arg=i;while(1){var a=r.delegate;if(a){var c=S(a,r);if(c){if(c===d)continue;return c}}if("next"===r.method)r.sent=r._sent=r.arg;else if("throw"===r.method){if(n===l)throw n=v,r.arg;r.dispatchException(r.arg)}else"return"===r.method&&r.abrupt("return",r.arg);n=p;var s=b(t,e,r);if("normal"===s.type){if(n=r.done?v:f,s.arg===d)continue;return{value:s.arg,done:r.done}}"throw"===s.type&&(n=v,r.method="throw",r.arg=s.arg)}}}function S(t,e){var n=t.iterator[e.method];if(n===r){if(e.delegate=null,"throw"===e.method){if(t.iterator.return&&(e.method="return",e.arg=r,S(t,e),"throw"===e.method))return d;e.method="throw",e.arg=new TypeError("The iterator does not provide a 'throw' method")}return d}var o=b(n,t.iterator,e.arg);if("throw"===o.type)return e.method="throw",e.arg=o.arg,e.delegate=null,d;var i=o.arg;return i?i.done?(e[t.resultName]=i.value,e.next=t.nextLoc,"return"!==e.method&&(e.method="next",e.arg=r),e.delegate=null,d):i:(e.method="throw",e.arg=new TypeError("iterator result is not an object"),e.delegate=null,d)}function j(t){var e={tryLoc:t[0]};1 in t&&(e.catchLoc=t[1]),2 in t&&(e.finallyLoc=t[2],e.afterLoc=t[3]),this.tryEntries.push(e)}function N(t){var e=t.completion||{};e.type="normal",delete e.arg,t.completion=e}function G(t){this.tryEntries=[{tryLoc:"root"}],t.forEach(j,this),this.reset(!0)}function C(t){if(t){var e=t[a];if(e)return e.call(t);if("function"===typeof t.next)return t;if(!isNaN(t.length)){var n=-1,i=function e(){while(++n<t.length)if(o.call(t,n))return e.value=t[n],e.done=!1,e;return e.value=r,e.done=!0,e};return i.next=i}}return{next:I}}function I(){return{value:r,done:!0}}}(function(){return this}()||Function("return this")())},c878:function(t,e,r){},cf05:function(t,e,r){t.exports=r.p+"img/logo.37942b4b.png"},d45b:function(t,e,r){"use strict";r("c878")}}]);