(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["ax0"],{"4ade":function(e,o,s){},"578a":function(e,o,s){"use strict";s.r(o);var t=function(){var e=this,o=e._self._c;return o("div",{staticClass:"login_container"},[e._m(0),o("div",{staticClass:"login_box"},[e._m(1),o("el-divider"),o("el-form",{ref:"loginFormRef",staticClass:"login_form",attrs:{rules:e.loginFormRules,model:e.loginForm,"label-width":"0px"}},[o("el-form-item",{attrs:{prop:"username"}},[o("el-input",{attrs:{"prefix-icon":"el-icon-s-custom",placeholder:"请输入用户名"},model:{value:e.loginForm.username,callback:function(o){e.$set(e.loginForm,"username",o)},expression:"loginForm.username"}})],1),o("el-form-item",{attrs:{prop:"password"}},[o("el-input",{attrs:{type:"password","prefix-icon":"el-icon-lock",placeholder:"请输入密码"},model:{value:e.loginForm.password,callback:function(o){e.$set(e.loginForm,"password",o)},expression:"loginForm.password"}})],1),o("el-form-item",[o("el-checkbox",{attrs:{label:"记住用户名"},model:{value:e.keepuser,callback:function(o){e.keepuser=o},expression:"keepuser"}})],1),o("el-form-item",[o("el-button",{staticClass:"login-button",attrs:{type:"primary"},on:{click:e.doLogin}},[e._v("登录")])],1)],1)],1)])},r=[function(){var e=this,o=e._self._c;return o("div",{staticClass:"login_logo"},[o("img",{attrs:{src:s("cf05")}})])},function(){var e=this,o=e._self._c;return o("div",{staticClass:"login_title"},[o("span",[e._v("系统登录")])])}],i=(s("14d9"),{data(){return{loginForm:{username:"admin",password:"admin"},keepuser:!0,loginFormRules:{username:[{required:!0,message:" ",trigger:"blur"}],password:[{required:!0,message:" ",trigger:"blur"}]}}},created(){console.log("web app version: V1.21.05.2"),this.doInit()},methods:{doLogin(){this.$refs.loginFormRef.validate(async e=>{if(console.log("username: ",this.loginForm.username),!e)return!1;window.localStorage.setItem("keepuser",this.keepuser),this.keepuser&&window.localStorage.setItem("username",this.loginForm.username);const{data:o}=await this.$http.post("login",this.loginForm);if(console.log("post login return: ",o),200!==o.meta.status)return this.$message.error("登录失败");this.$message.success("登录成功"),window.sessionStorage.setItem("token",o.data.token),window.sessionStorage.setItem("reloadFlag","true"),this.$router.push("/home")})},doInit(){this.keepuser="true"===window.localStorage.getItem("keepuser"),console.log("keepuser: "+this.keepuser),this.keepuser&&(this.loginForm.username=window.localStorage.getItem("username"),console.log("username: "+this.loginForm.username))}}}),n=i,l=(s("f8fc"),s("2877")),a=Object(l["a"])(n,t,r,!1,null,"4c3b19c4",null);o["default"]=a.exports},cf05:function(e,o,s){e.exports=s.p+"img/logo.37942b4b.png"},f8fc:function(e,o,s){"use strict";s("4ade")}}]);